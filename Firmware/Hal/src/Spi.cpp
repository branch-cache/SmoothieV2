#include "Spi.h"
#include "Hal_pin.h"

#include "stm32h7xx_hal.h"
#ifdef USE_FULL_LL_DRIVER
#include "stm32h7xx_ll_rcc.h"
#endif

#include <stdio.h>
#include <cstring>


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#if defined (BOARD_IKOSYBOT)
/* Define SPI1 for 1st SPI */
#define SPI1x                              SPI1
#define SPI1x_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define DMA1_CLK_ENABLE()                 __HAL_RCC_DMA2_CLK_ENABLE()
#define SPI1x_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPI1x_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1x_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1x_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPI1x_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()
#define SPI1x_IRQn                        SPI1_IRQn
#define SPI1x_IRQHandler                  SPI1_IRQHandler

/* Definition SPI1 Pins */
#define SPI1x_SCK_PIN                     GPIO_PIN_9
#define SPI1x_SCK_GPIO_PORT               GPIOA
#define SPI1x_SCK_AF                      GPIO_AF5_SPI1
#define SPI1x_MISO_PIN                    GPIO_PIN_14
#define SPI1x_MISO_GPIO_PORT              GPIOB
#define SPI1x_MISO_AF                     GPIO_AF5_SPI1
#define SPI1x_MOSI_PIN                    GPIO_PIN_15
#define SPI1x_MOSI_GPIO_PORT              GPIOB
#define SPI1x_MOSI_AF                     GPIO_AF5_SPI1

#else
/* Define SPI1 for 1st SPI */
#define SPI1x                              SPI1
#define SPI1x_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define DMA1_CLK_ENABLE()                 __HAL_RCC_DMA2_CLK_ENABLE()
#define SPI1x_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPI1x_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPI1x_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI1x_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPI1x_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()
#define SPI1x_IRQn                        SPI1_IRQn
#define SPI1x_IRQHandler                  SPI1_IRQHandler

/* Definition SPI1 Pins */
#define SPI1x_SCK_PIN                     GPIO_PIN_5
#define SPI1x_SCK_GPIO_PORT               GPIOA
#define SPI1x_SCK_AF                      GPIO_AF5_SPI1
#define SPI1x_MISO_PIN                    GPIO_PIN_6
#define SPI1x_MISO_GPIO_PORT              GPIOA
#define SPI1x_MISO_AF                     GPIO_AF5_SPI1
#define SPI1x_MOSI_PIN                    GPIO_PIN_5
#define SPI1x_MOSI_GPIO_PORT              GPIOB
#define SPI1x_MOSI_AF                     GPIO_AF5_SPI1
#endif

#ifdef BOARD_DEVEBOX
// Define SPI2 for 2nd SPI
#define SPI2x                             SPI2
#define SPI2x_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define DMA1_CLK_ENABLE()                 __HAL_RCC_DMA2_CLK_ENABLE()
#define SPI2x_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2x_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2x_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI2x_IRQn                        SPI2_IRQn
#define SPI2x_IRQHandler                  SPI2_IRQHandler

#define SPI2x_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPI2x_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

// Definition for SPI2 Pins
#define SPI2x_SCK_PIN                     GPIO_PIN_13
#define SPI2x_SCK_GPIO_PORT               GPIOB
#define SPI2x_SCK_AF                      GPIO_AF5_SPI2
#define SPI2x_MISO_PIN                    GPIO_PIN_14
#define SPI2x_MISO_GPIO_PORT              GPIOB
#define SPI2x_MISO_AF                     GPIO_AF5_SPI2
#define SPI2x_MOSI_PIN                    GPIO_PIN_15
#define SPI2x_MOSI_GPIO_PORT              GPIOB
#define SPI2x_MOSI_AF                     GPIO_AF5_SPI2

#else

/* Define SPI4 for 2nd SPI*/
#define SPI2x                             SPI4
#define SPI2x_CLK_ENABLE()                __HAL_RCC_SPI4_CLK_ENABLE()
#define DMA1_CLK_ENABLE()                 __HAL_RCC_DMA2_CLK_ENABLE()
#define RCC_PERIPHCLK_SPI2x               RCC_PERIPHCLK_SPI4
#define SPI2x_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define SPI2x_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define SPI2x_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define SPI2x_IRQn                        SPI4_IRQn
#define SPI2x_IRQHandler                  SPI4_IRQHandler

#define SPI2x_FORCE_RESET()               __HAL_RCC_SPI4_FORCE_RESET()
#define SPI2x_RELEASE_RESET()             __HAL_RCC_SPI4_RELEASE_RESET()

/* Definition for SPI4 Pins */
#define SPI2x_SCK_PIN                     GPIO_PIN_12
#define SPI2x_SCK_GPIO_PORT               GPIOE
#define SPI2x_SCK_AF                      GPIO_AF5_SPI4
#define SPI2x_MISO_PIN                    GPIO_PIN_5
#define SPI2x_MISO_GPIO_PORT              GPIOE
#define SPI2x_MISO_AF                     GPIO_AF5_SPI4
#define SPI2x_MOSI_PIN                    GPIO_PIN_6
#define SPI2x_MOSI_GPIO_PORT              GPIOE
#define SPI2x_MOSI_AF                     GPIO_AF5_SPI4
#endif

SPI *SPI::spi_channel[2];
// static
SPI *SPI::getInstance(int channel)
{
    if(channel >= 2) return nullptr;
    if(spi_channel[channel] == nullptr) {
        spi_channel[channel] = new SPI(channel);
    }

    return spi_channel[channel];
}
void SPI::deleteInstance(int channel)
{
    if(channel >= 2) return;
    if(spi_channel[channel] == nullptr) {
        return;
    }

    delete spi_channel[channel];
    spi_channel[channel] = nullptr;
}

SPI::SPI(int channel)
{
    _valid = false;
    _channel = channel;
    // setup a mutex to stop concurrent access
    mutex= xSemaphoreCreateMutex();
}

bool SPI::init(int bits, int mode, int frequency)
{
    if(_valid) {
        if(bits != _bits || mode != _mode || frequency != _hz) {
            printf("ERROR: SPI channel %d, already set with different parameters\n", _channel);
            return false;
        }
        return true;
    }

    _bits = bits;
    _mode = mode;
    _hz = frequency;

#if 0
    // TODO maybe only do this if requested frequency is below 750KHz
    // change the clock the SPI uses
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if(_channel == 1) {
        // set to 10MHz for SPI
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
        PeriphClkInitStruct.PLL2.PLL2M = 25;
        PeriphClkInitStruct.PLL2.PLL2N = 150;
        PeriphClkInitStruct.PLL2.PLL2P = 15;
        PeriphClkInitStruct.PLL2.PLL2Q = 15;
        PeriphClkInitStruct.PLL2.PLL2R = 2;
        PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
        PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
        PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
        PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        {
            printf("HAL_SPI_MspInit: HAL_RCCEx_PeriphCLKConfig failed\n");
            return false;
        }

        /* SPI4 clock enable */
        __HAL_RCC_SPI4_CLK_ENABLE();

    }else if(_channel == 0){
        // set to 10MHz for SPI
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
        #if defined(BOARD_NUCLEO) || defined(BOARD_IKOSYBOT)
        PeriphClkInitStruct.PLL2.PLL2M = 1;
        PeriphClkInitStruct.PLL2.PLL2N = 20;
        PeriphClkInitStruct.PLL2.PLL2P = 16;
        PeriphClkInitStruct.PLL2.PLL2Q = 16;
        PeriphClkInitStruct.PLL2.PLL2R = 2;
        #else
        PeriphClkInitStruct.PLL2.PLL2M = 25;
        PeriphClkInitStruct.PLL2.PLL2N = 150;
        PeriphClkInitStruct.PLL2.PLL2P = 15;
        PeriphClkInitStruct.PLL2.PLL2Q = 15;
        PeriphClkInitStruct.PLL2.PLL2R = 2;
        #endif
        PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
        PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
        PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            printf("HAL_SPI_MspInit: HAL_RCCEx_PeriphCLKConfig failed\n");
            return false;
        }
        SPI1x_CLK_ENABLE();
    }
    spi_hz= 10000000; // 10 MHz
#endif

    uint32_t spi_hz;
#ifdef USE_FULL_LL_DRIVER
    spi_hz = LL_RCC_GetSPIClockFreq(LL_RCC_SPI123_CLKSOURCE);
    printf("DEBUG: SPI clk source: %lu hz\n", spi_hz);
    uint32_t f = (uint32_t) (spi_hz / frequency);
#else
    // divisor needed for requested frequency
    spi_hz= SystemCoreClock / 2;
    uint32_t f = (uint32_t) spi_hz / frequency;
#endif

    uint32_t psc = 0;
    // find nearest prescaler
    if(f <= 2) { psc = SPI_BAUDRATEPRESCALER_2; f = 2; }
    else if(f <= 4) { psc = SPI_BAUDRATEPRESCALER_4; f = 4; }
    else if(f <= 8) { psc = SPI_BAUDRATEPRESCALER_8; f = 8; }
    else if(f <= 16) { psc = SPI_BAUDRATEPRESCALER_16; f = 16; }
    else if(f <= 32) { psc = SPI_BAUDRATEPRESCALER_32; f = 32; }
    else if(f <= 64) { psc = SPI_BAUDRATEPRESCALER_64; f = 64; }
    else if(f <= 128) { psc = SPI_BAUDRATEPRESCALER_128; f = 128; }
    else if(f <= 256) { psc = SPI_BAUDRATEPRESCALER_256; f = 256; }
    else {
        f = 256;
        psc = SPI_BAUDRATEPRESCALER_256;
        printf("WARNING: cannot get SPI frequency this low, set to: %lu hz\n", spi_hz / f);
    }
    printf("DEBUG: SPI frequency set to: %lu hz\n", spi_hz / f);

    /* Set the SPI parameters */
    SPI_HandleTypeDef SpiHandle{0};
    SpiHandle.Instance               = _channel == 0 ? SPI1x : SPI2x;
    SpiHandle.Init.BaudRatePrescaler = psc;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = (mode & 1) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = (mode & 2) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = bits - 1;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    SpiHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommended setting to avoid glitches */
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;

    if(HAL_SPI_Init(&SpiHandle) != HAL_OK) {
        return false;
    }
    _hspi = malloc(sizeof(SPI_HandleTypeDef));
    memcpy(_hspi, &SpiHandle, sizeof(SPI_HandleTypeDef));

   _valid = true;

    return true;
}

SPI::~SPI()
{
    vSemaphoreDelete(mutex);
    if(_valid) {
        HAL_SPI_DeInit((SPI_HandleTypeDef*)_hspi);
        free(_hspi);
    }
}

bool SPI::begin_transaction(uint32_t tmoms)
{
    uint32_t t= pdMS_TO_TICKS(tmoms);
    return xSemaphoreTake(mutex, t) == pdTRUE;
}

void SPI::end_transaction()
{
    xSemaphoreGive(mutex);
}

// writes and reads number of _bits sized words
bool SPI::write_read(void *wvalue, void *rvalue, uint32_t n)
{
    if(wvalue == nullptr || rvalue == nullptr) return false;

#if 1
    if(HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)_hspi, (uint8_t*)wvalue, (uint8_t*)rvalue, n, 1000) != HAL_OK) {
        // Transfer error in transmission process
        return false;
    }
#else
    // send one byte at a time
    for (uint32_t i = 0; i < n; ++i) {
        if(HAL_SPI_TransmitReceive((SPI_HandleTypeDef*)_hspi, ((uint8_t*)wvalue)+i, ((uint8_t*)rvalue)+i, 1, 1000) != HAL_OK) {
            // Transfer error in transmission process
            return false;
        }
    }
#endif
    // printf("Sent: "); for (uint32_t i = 0; i < n; ++i) printf("%02X ", ((char*)wvalue)[i]); printf("\n");
    // printf("Rcvd: "); for (uint32_t i = 0; i < n; ++i) printf("%02X ", ((char*)rvalue)[i]); printf("\n");

    return true;
}

/**
  * @brief SPI MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param hspi: SPI handle pointer
  * @retval None
  */
extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef  GPIO_InitStruct{0};

    if (hspi->Instance == SPI1x) {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        SPI1x_SCK_GPIO_CLK_ENABLE();
        SPI1x_MISO_GPIO_CLK_ENABLE();
        SPI1x_MOSI_GPIO_CLK_ENABLE();
        /* Enable SPI1 clock */
        SPI1x_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* SPI SCK GPIO pin configuration  */
        GPIO_InitStruct.Pin       = SPI1x_SCK_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = (SPI::spi_channel[0]->get_mode() & 2) ? GPIO_PULLUP : GPIO_PULLDOWN;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = SPI1x_SCK_AF;
        HAL_GPIO_Init(SPI1x_SCK_GPIO_PORT, &GPIO_InitStruct);

        /* SPI MISO GPIO pin configuration  */
        GPIO_InitStruct.Pin = SPI1x_MISO_PIN;
        GPIO_InitStruct.Alternate = SPI1x_MISO_AF;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        HAL_GPIO_Init(SPI1x_MISO_GPIO_PORT, &GPIO_InitStruct);

        /* SPI MOSI GPIO pin configuration  */
        GPIO_InitStruct.Pin = SPI1x_MOSI_PIN;
        GPIO_InitStruct.Alternate = SPI1x_MOSI_AF;
        HAL_GPIO_Init(SPI1x_MOSI_GPIO_PORT, &GPIO_InitStruct);

        /*##-5- Configure the NVIC for SPI #########################################*/
        /* NVIC configuration for SPI transfer complete interrupt (SPI1) */
        // NVIC_SetPriority(SPI1x_IRQn, 5);
        // NVIC_EnableIRQ(SPI1x_IRQn);

        allocate_hal_pin(SPI1x_SCK_GPIO_PORT, SPI1x_SCK_PIN);
        allocate_hal_pin(SPI1x_MISO_GPIO_PORT, SPI1x_MISO_PIN);
        allocate_hal_pin(SPI1x_MOSI_GPIO_PORT, SPI1x_MOSI_PIN);

    } else if (hspi->Instance == SPI2x) {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        SPI2x_SCK_GPIO_CLK_ENABLE();
        SPI2x_MISO_GPIO_CLK_ENABLE();
        SPI2x_MOSI_GPIO_CLK_ENABLE();
        /* Enable SPI2 clock */
        SPI2x_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* SPI SCK GPIO pin configuration  */
        GPIO_InitStruct.Pin       = SPI2x_SCK_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = (SPI::spi_channel[1]->get_mode() & 2) ? GPIO_PULLUP : GPIO_PULLDOWN;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = SPI2x_SCK_AF;
        HAL_GPIO_Init(SPI2x_SCK_GPIO_PORT, &GPIO_InitStruct);

        /* SPI MISO GPIO pin configuration  */
        GPIO_InitStruct.Pin = SPI2x_MISO_PIN;
        GPIO_InitStruct.Alternate = SPI2x_MISO_AF;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        HAL_GPIO_Init(SPI2x_MISO_GPIO_PORT, &GPIO_InitStruct);

        /* SPI MOSI GPIO pin configuration  */
        GPIO_InitStruct.Pin = SPI2x_MOSI_PIN;
        GPIO_InitStruct.Alternate = SPI2x_MOSI_AF;
        HAL_GPIO_Init(SPI2x_MOSI_GPIO_PORT, &GPIO_InitStruct);

        /*##-5- Configure the NVIC for SPI #########################################*/
        /* NVIC configuration for SPI transfer complete interrupt (SPI1) */
        // NVIC_SetPriority(SPI2x_IRQn, 5);
        // NVIC_EnableIRQ(SPI2x_IRQn);
        allocate_hal_pin(SPI2x_SCK_GPIO_PORT, SPI2x_SCK_PIN);
        allocate_hal_pin(SPI2x_MISO_GPIO_PORT, SPI2x_MISO_PIN);
        allocate_hal_pin(SPI2x_MOSI_GPIO_PORT, SPI2x_MOSI_PIN);
    }
}

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
extern "C" void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1) {
        /*##-1- Reset peripherals ##################################################*/
        SPI1x_FORCE_RESET();
        SPI1x_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks ################################*/
        /* Deconfigure SPI SCK */
        HAL_GPIO_DeInit(SPI1x_SCK_GPIO_PORT, SPI1x_SCK_PIN);
        /* Deconfigure SPI MISO */
        HAL_GPIO_DeInit(SPI1x_MISO_GPIO_PORT, SPI1x_MISO_PIN);
        /* Deconfigure SPI MOSI */
        HAL_GPIO_DeInit(SPI1x_MOSI_GPIO_PORT, SPI1x_MOSI_PIN);

        /*##-5- Disable the NVIC for SPI ###########################################*/
        NVIC_DisableIRQ(SPI1x_IRQn);

    } else if(hspi->Instance == SPI2) {
        /*##-1- Reset peripherals ##################################################*/
        SPI2x_FORCE_RESET();
        SPI2x_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks ################################*/
        /* Deconfigure SPI SCK */
        HAL_GPIO_DeInit(SPI2x_SCK_GPIO_PORT, SPI2x_SCK_PIN);
        /* Deconfigure SPI MISO */
        HAL_GPIO_DeInit(SPI2x_MISO_GPIO_PORT, SPI2x_MISO_PIN);
        /* Deconfigure SPI MOSI */
        HAL_GPIO_DeInit(SPI2x_MOSI_GPIO_PORT, SPI2x_MOSI_PIN);

        /*##-5- Disable the NVIC for SPI ###########################################*/
        NVIC_DisableIRQ(SPI2x_IRQn);
    }
}

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of IT TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    // wTransferState = TRANSFER_ERROR;
}

/**
  * @brief  This function handles SPIx interrupt request.
  * @param  None
  * @retval None
  */
extern "C" void SPI1x_IRQHandler(void)
{
    HAL_SPI_IRQHandler((SPI_HandleTypeDef*)SPI::spi_channel[0]->get_hspi());
}

extern "C" void SPI2x_IRQHandler(void)
{
    HAL_SPI_IRQHandler((SPI_HandleTypeDef*)SPI::spi_channel[1]->get_hspi());
}
