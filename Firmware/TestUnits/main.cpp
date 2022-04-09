#include <stdio.h>

#include <vector>
#include <tuple>
#include <functional>

#include <malloc.h>
#include <string.h>

#include "../Unity/src/unity.h"
#include "TestRegistry.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "OutputStream.h"
#include "MessageQueue.h"
#include "SlowTicker.h"

#include "uart_debug.h"

#include "benchmark_timer.h"

//#define TESTCOMMS

// // place holder
bool dispatch_line(OutputStream& os, const char *line)
{
    return true;
}

static std::function<void(void)> setup_fnc;
void setUp(void)
{
    if(setup_fnc)
        setup_fnc();
}

static std::function<void(void)> teardown_fnc;
void tearDown(void)
{
    if(teardown_fnc)
        teardown_fnc();
}

static std::function<void(void)> test_wrapper_fnc;
static void test_wrapper(void)
{
    test_wrapper_fnc();
}

static int test_runner(void)
{
    auto tests = TestRegistry::instance().get_tests();
    printf("There are %d registered tests...\n", tests.size());
    for(auto& i : tests) {
        printf("  %s\n", std::get<1>(i));
    }

    UnityBegin("TestUnits");

    for(auto i : tests) {
        TestBase *fnc = std::get<0>(i);
        const char *name = std::get<1>(i);
        int ln = std::get<2>(i);
        Unity.TestFile = std::get<3>(i);
        test_wrapper_fnc = std::bind(&TestBase::test, fnc);
        bool st = std::get<4>(i);
        if(st) {
            setup_fnc = std::bind(&TestBase::setUp, fnc);
            teardown_fnc = std::bind(&TestBase::tearDown, fnc);
        } else {
            setup_fnc = nullptr;
            teardown_fnc = nullptr;
        }

        UnityDefaultTestRun(test_wrapper, name, ln);

        // if we get any errors stop here
        if(Unity.TestFailures > 0) break;
    }

    return (UnityEnd());
}

static int run_tests()
{
    printf("Starting tests...\n");
    int ret = test_runner();
    printf("Done\n");
    return ret;
}

void safe_sleep(uint32_t ms)
{
    // here we need to sleep (and yield) for 10ms then check if we need to handle the query command
    TickType_t delayms = pdMS_TO_TICKS(10); // 10 ms sleep
    while(ms > 0) {
        vTaskDelay(delayms);
        if(ms > 10) {
            ms -= 10;
        } else {
            break;
        }
    }
}

void print_to_all_consoles(const char *str)
{
    printf("%s", str);
}

extern "C" void vRunTestsTask(void *pvParameters)
{
    run_tests();

    //vTaskGetTaskState();
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("High water mark= %lu\n", uxHighWaterMark);

    TickType_t delayms = pdMS_TO_TICKS(1000);
    for(;;) {
        vTaskDelay(delayms);
    }
    vTaskDelete( NULL );
}

#ifdef TESTCOMMS
extern "C" size_t write_cdc(int, const char *buf, size_t len);
extern "C" size_t read_cdc(int, char *buf, size_t len);
extern "C" int setup_cdc();
extern "C" int vcom_is_connected(int);
extern "C" uint32_t vcom_get_dropped_bytes(int);

static std::function<size_t(char *, size_t)> capture_fnc = nullptr;

int maxmq = 20;
uint32_t timeouts = 0;
extern "C" void usbComTask(void *pvParameters)
{
    char linebuf[MAX_LINE_LENGTH];
    size_t linecnt = 0;
    char rxBuff[1024];
    bool discard = false;

    // setup the USB CDC
    if(setup_cdc() == 0) {
        printf("ERROR: setup_cdc failed\n");
        return;
    }

    // when we get the first connection it sends a one byte message to wake us up
    // it will block here until a connection is available
    size_t n = read_cdc(0, rxBuff, 1);
    if(rxBuff[0] != 1 || !vcom_is_connected(0)) {
        printf("Unexpected CDC startup frame: %d\n", n);
    }
    printf("CDC connected\n");

    static OutputStream theos([](const char *buf, size_t len) { return write_cdc(0, buf, len); });
    theos.puts("Welcome to TestUnits\nok\n");

    while(1) {
        // we read as much as we can, process it into lines and send it to the dispatch thread
        // certain characters are sent immediately the rest wait for end of line
        size_t rdCnt = read_cdc(0, rxBuff, sizeof(rxBuff));
        uint32_t db;
        if((db=vcom_get_dropped_bytes(0)) > 0) {
            printf("WARNING got dropped bytes: %lu\n", db);
        }

        if(capture_fnc) {
            // returns used characters, if used < rdCnt then it is done
            // so we reset it and give the rest to the normal processing
            // if used > rdCnt then it is done but we used all the characters
            size_t used = capture_fnc(rxBuff, rdCnt);
            if(used < rdCnt) {
                capture_fnc = nullptr;
                if(used > 0) {
                    memmove(rxBuff, &rxBuff[used], rdCnt - used);
                    rdCnt = rdCnt - used;
                }
                // drop through to process the rest

            } else if(used > rdCnt) {
                // we used all the data but we are done now
                capture_fnc = nullptr;
                continue;
            } else {
                continue;
            }
        }

        // process line character by character (pretty slow)
        for (size_t i = 0; i < rdCnt; ++i) {
            linebuf[linecnt] = rxBuff[i];

            // the following are single character commands that are dispatched immediately
            if(linebuf[linecnt] == 24) { // ^X
                // discard all recieved data
                linebuf[linecnt + 1] = '\0'; // null terminate
                send_message_queue(&linebuf[linecnt], &theos);
                linecnt = 0;
                discard = false;
                break;
            } else if(linebuf[linecnt] == '?') {
                linebuf[linecnt + 1] = '\0'; // null terminate
                send_message_queue(&linebuf[linecnt], &theos);
            } else if(linebuf[linecnt] == '!') {
                linebuf[linecnt + 1] = '\0'; // null terminate
                send_message_queue(&linebuf[linecnt], &theos);
            } else if(linebuf[linecnt] == '~') {
                linebuf[linecnt + 1] = '\0'; // null terminate
                send_message_queue(&linebuf[linecnt], &theos);
                // end of immediate commands

            } else if(discard) {
                // we discard long lines until we get the newline
                if(linebuf[linecnt] == '\n') discard = false;

            } else if(linecnt >= sizeof(linebuf) - 1) {
                // discard long lines
                discard = true;
                linecnt = 0;
                printf("Discarding long line\n");

            } else if(linebuf[linecnt] == '\n') {
                linebuf[linecnt] = '\0'; // remove the \n and nul terminate
                send_message_queue(linebuf, &theos);
                linecnt = 0;
                maxmq = std::min(get_message_queue_space(), maxmq);

            } else if(linebuf[linecnt] == '\r') {
                // ignore CR
                continue;

            } else if(linebuf[linecnt] == 8 || linebuf[linecnt] == 127) { // BS or DEL
                if(linecnt > 0) --linecnt;

            } else {
                ++linecnt;
            }
        }
    }
}

TickType_t delayms = pdMS_TO_TICKS(1);
#include "md5.h"
// test fast streaming from host
// reception is mostly in comms thread
void fast_download_test(OutputStream *os)
{
    os->puts("Starting fast download test\nok\n");
    size_t cnt = 0;
    MD5 md5;
    volatile bool done = false;

    // capture any input, return used number of characters
    // if we return < n or > n then capture_fnc is terminated
    // < n means process the rest of the buffer normally
    // Note that this call is in comms thread not command thread
    capture_fnc = ([&md5, &done, &cnt](char *b, size_t n) {
        size_t s;
        // if we get a 0x04 then that is the end of the stream
        char *p = (char *)memchr(b, 4, n); // see if we have termination character
        if(p != NULL) {
            // we do
            s = p - b; // number of characters before the terminator
            if(p == b + n - 1) {
                // we used the entire buffer as terminator is at end
                n = n + 1; // mark it so we terminate but used all the buffer
            } else {
                // we used partial buffer
                n = s + 1;
            }

        } else {
            // we can use entire buffer and keep going
            s = n;
        }

        if(s > 0) {
            //printf("Got %d bytes\n", s);
            md5.update(b, s);
            cnt += s;
        }

        // do this last as we may get preempted
        if(p != NULL) done = true;

        return n;
    });

    while(!done) {
        vTaskDelay(delayms);
    }

    os->printf("md5: %s, cnt: %u\n", md5.finalize().hexdigest().c_str(), cnt);
    os->puts("download test complete\n");
}

void send_test(OutputStream *os)
{
    os->puts("Starting send test...\n");
    const int n = 4096;
    os->printf("Sending %d bytes...\n", n);
    char *buf = (char *)malloc(n + 1);
    //assert(buf != NULL);
    buf[0] = 0;
    for (int i = 0; i < n / 16; ++i) {
        strcat(buf, "123456789ABCDEF\n");
    }
    write_cdc(0, buf, n);
    free(buf);

    // now send lots of little lines
    os->printf("Sending 1000 lines...\n", n);
    buf = strdup("ok\n");
    for (int i = 0; i < 1000; ++i) {
        write_cdc(0, buf, strlen(buf));
    }
    free(buf);
}

// this would be the command thread in the firmware
extern "C" void dispatch(void *pvParameters)
{
    char *line;
    OutputStream *os;
    bool download_mode = false;
    MD5 md5;
    size_t cnt = 0, lcnt = 0;
    while(1) {
        // now read lines and dispatch them
        if( receive_message_queue(&line, &os) ) {
            // if we are in the download mode (simulating M28)
            // then just md5 the data until we are done
            if(download_mode) {
                if(strcmp(line, "M29") == 0) {
                    download_mode = false;
                    os->printf("Done saving file.\nok\n");
                    printf("md5: %s, cnt: %u, timeouts: %lu, maxmq: %d\n", md5.finalize().hexdigest().c_str(), cnt, timeouts, maxmq);
                    continue;
                }
                md5.update(line, strlen(line));
                md5.update("\n", 1);
                cnt += (strlen(line) + 1);
                ++lcnt;
                os->puts("ok\n");
                continue;
            }

            // got line
            if(strlen(line) == 1) {
                switch(line[0]) {
                    case 24: os->printf("Got KILL\n"); break;
                    case '?': os->printf("Got Query\n"); break;
                    case '!': os->printf("Got Hold\n"); break;
                    case '~': os->printf("Got Release\n"); break;
                    default: os->printf("Got 1 char line: %s\n", line);
                }

            } else {

                if(strcmp(line, "mem") == 0) {
                    char pcWriteBuffer[500];
                    vTaskList( pcWriteBuffer );
                    os->puts(pcWriteBuffer);
                    // os->puts("\n\n");
                    // vTaskGetRunTimeStats(pcWriteBuffer);
                    // os->puts(pcWriteBuffer);

                    struct mallinfo mi = mallinfo();
                    os->printf("\n\nfree malloc memory= %d, free sbrk memory= %d, Total free= %d\n", mi.fordblks, xPortGetFreeHeapSize() - mi.fordblks, xPortGetFreeHeapSize());

                } else if(strcmp(line, "rxtest") == 0) {
                    // do a download test
                    fast_download_test(os);

                } else if(strcmp(line, "txtest") == 0) {
                    // do a USB send test
                    send_test(os);

                } else if(strncmp(line, "M28 ", 4) == 0) {
                    download_mode = true;
                    cnt = 0;
                    lcnt = 0;
                    md5.reinit();
                    timeouts = 0;
                    os->printf("Writing to file: SIMULATION\n");

                } else {
                    os->printf("Got line: %s\n", line);
                }

                os->puts("ok\n");
            }
        }
    }
}
#endif

extern "C" void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}

extern "C" void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    __disable_irq();
    __asm volatile ("bkpt #0");
    for( ;; );
}

extern "C" void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

extern "C" void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    __disable_irq();
    __asm volatile ("bkpt #0");
    for( ;; );
}

extern "C" void HardFault_Handler(void)
{
    __asm("bkpt #0");
    for( ;; );
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
extern "C" void assert_failed(uint8_t* file, uint32_t line)
{
    printf("ERROR: HAL assert failed: file %s on line %lu\n", file, line);

    //__disable_irq();
    __asm volatile ("bkpt #0");
    /* Infinite loop */
    while (1) {
    }
}
#endif

// #define _ramfunc_ __attribute__ ((section(".ramfunctions"),long_call,noinline))
// _ramfunc_ void TIMER0_IRQHandler(void)
// {
//     printf("hello\n");
// }

extern "C" void setup_xprintf();
extern "C" void main_system_setup();
extern "C" void print_clocks();
std::string get_mcu();
int main()
{
    // allows std::cout to work again (not sure why)
    std::ios_base::sync_with_stdio(false);

    // setup clock and caches etc (in HAL)
    main_system_setup();

    benchmark_timer_init();

    setup_xprintf();

    if(setup_uart() <= 0) {
        puts("FATAL: UART setup failed\n");
        __asm volatile ("bkpt #0");
    }

    printf("%s on %s\n", get_mcu().c_str(), BUILD_TARGET);
    printf("MCU clock rate= %lu Hz\n", SystemCoreClock);
    print_clocks();

    xTaskCreate(vRunTestsTask, "CommandThread", 1024, /* *4 as 32bit words */
                NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

#ifdef TESTCOMMS
    // create queue for dispatch of lines, can be sent to by several tasks
    if(!create_message_queue()) {
        // Failed to create the queue.
        printf("ERROR: failed to create dispatch queue\n");
        __asm volatile ("bkpt #0");
    }

    xTaskCreate(usbComTask, "usbComTask", 1024, NULL, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *) NULL);
    xTaskCreate(dispatch, "dispatch", 512, NULL, (tskIDLE_PRIORITY + 4UL), NULL);
#endif

    struct mallinfo mi = mallinfo();
    printf("free malloc memory= %d, free sbrk memory= %d, Total free= %d\n", mi.fordblks, xPortGetFreeHeapSize() - mi.fordblks, xPortGetFreeHeapSize());

    /* Start the scheduler */
    vTaskStartScheduler();

    // should never reach here
    __asm volatile ("bkpt #0");
}

#ifdef configASSERT
void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    volatile uint32_t ulBlockVariable = 0UL;
    volatile const char *pcAssertedFileName;
    volatile int iAssertedErrno;
    volatile uint32_t ulAssertedLine;

    ulAssertedLine = ulLine;
    iAssertedErrno = errno;
    pcAssertedFileName = strrchr( pcFile, '/' );

    /* These variables are set so they can be viewed in the debugger, but are
    not used in the code - the following lines just remove the compiler warning
    about this. */
    ( void ) ulAssertedLine;
    ( void ) iAssertedErrno;

    if( pcAssertedFileName == 0 ) {
        pcAssertedFileName = strrchr( pcFile, '\\' );
    }
    if( pcAssertedFileName != NULL ) {
        pcAssertedFileName++;
    } else {
        pcAssertedFileName = pcFile;
    }
    printf("ERROR: vAssertCalled( %s, %ld\n", pcFile, ulLine);

    // return;

    /* Setting ulBlockVariable to a non-zero value in the debugger will allow
    this function to be exited. */
    taskDISABLE_INTERRUPTS();
    {
        __asm volatile ("bkpt #0");
        while( ulBlockVariable == 0UL ) {
            __asm volatile( "NOP" );
        }
    }
    taskENABLE_INTERRUPTS();
}

using StartupFunc_t = std::function<void()>;
void register_startup(StartupFunc_t sf)
{
    // dummy
}

#endif
