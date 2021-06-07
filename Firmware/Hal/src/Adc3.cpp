#include "Adc3.h"

#include <cctype>
#include <limits>

#include "stm32h7xx_hal.h"
#include "Hal_pin.h"

/*
    ADC3    IN0 Single-ended        ADC3_INP0       PC2_C
    ADC3    IN6 Single-ended        ADC3_INP6       PF10
    ADC3    IN7 Single-ended        ADC3_INP7       PF8
    ADC3    IN8 Single-ended        ADC3_INP8       PF6
    ADC3    IN9 Single-ended        ADC3_INP9       PF4
    ADC3    IN16 Single-ended       ADC3_INP16      PH5
*/

Adc3 *Adc3::instance{nullptr};

static ADC_HandleTypeDef    AdcHandle;

static uint32_t adc_channel_lut[] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_8,
    ADC_CHANNEL_9,
    ADC_CHANNEL_16,
};

static uint32_t ADC3_Init(void)
{
    uint32_t ret = HAL_OK;

    AdcHandle.Instance = ADC3;

    if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
        return HAL_ERROR;
    }

    AdcHandle.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV6;          /* Asynchronous clock mode, input ADC clock divided by 6*/
    AdcHandle.Init.Resolution               = ADC_RESOLUTION_16B;            /* 16-bit resolution for converted data */
    AdcHandle.Init.ScanConvMode             = ENABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
    AdcHandle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
    AdcHandle.Init.LowPowerAutoWait         = DISABLE;                       /* Auto-delayed conversion feature disabled */
    AdcHandle.Init.ContinuousConvMode       = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
    AdcHandle.Init.NbrOfConversion          = 1;                             /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.DiscontinuousConvMode    = DISABLE;                       /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.NbrOfDiscConversion      = 1;                             /* Parameter discarded because sequencer is disabled */
    AdcHandle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
    AdcHandle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
    AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;         /* DR register used as output (DMA mode disabled) */
    AdcHandle.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;         /* Left shift of final results */
    AdcHandle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
    AdcHandle.Init.OversamplingMode         = DISABLE;                       /* Oversampling disable */

    /* Initialize ADC peripheral according to the passed parameters */
    if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
        return HAL_ERROR;
    }

    /* ### - 2 - Start calibration ############################################ */
    if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return HAL_ERROR;
    }

    return ret;
}

// one of ADC_CHANNEL_TEMPSENSOR ADC_CHANNEL_VREFINT ADC_CHANNEL_VBAT
static bool select_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig{0};
    sConfig.Channel                = channel;                    /* Sampled channel number */
    sConfig.Rank                   = ADC_REGULAR_RANK_1;         /* Rank of sampled channel number ADCx_CHANNEL */
    sConfig.SamplingTime           = ADC_SAMPLETIME_810CYCLES_5; /* Sampling time (number of clock cycles unit) */
    sConfig.SingleDiff             = ADC_SINGLE_ENDED;           /* Single input channel */
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;            /* No offset subtraction */
    sConfig.Offset                 = 0;                          /* Parameter discarded because offset correction is disabled */
    sConfig.OffsetRightShift       = DISABLE;                    /* No Right Offset Shift */
    sConfig.OffsetSignedSaturation = DISABLE;                    /* No Signed Saturation */
    if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
        return false;
    }
    return true;
}

static uint32_t ADC3_GetValue(void)
{
    if (HAL_ADC_PollForConversion(&AdcHandle, 10) == HAL_OK) {
        return HAL_ADC_GetValue(&AdcHandle);
    }else{
        return 0xFFFFFFFF;
    }
}

#define VREFANALOG_VOLTAGE  3300
static float ADC3_read_temp()
{
    select_channel(ADC_CHANNEL_TEMPSENSOR);
    HAL_ADC_Start(&AdcHandle);
    uint32_t value= ADC3_GetValue();
    HAL_ADC_Stop(&AdcHandle);
    if(value == 0xFFFFFFFF) return std::numeric_limits<float>::infinity();
    return (float)__LL_ADC_CALC_TEMPERATURE(VREFANALOG_VOLTAGE, value, ADC_RESOLUTION_16B);
}

Adc3::Adc3()
{
    if(HAL_OK == ADC3_Init()) {
        valid= true;
    }
}

Adc3::~Adc3()
{
    valid= false;
    HAL_ADC_DeInit(&AdcHandle);
}

bool Adc3::is_valid(int32_t ch) const
{
    if(!valid) return false;
    if((ch < 0 && (ch == -1 || ch == -2)) || (ch >= 0 && ch <= 5)) return true;
    return false;
}

float Adc3::read_temp()
{
    float t= ADC3_read_temp();
    return t;
}

float Adc3::read_voltage(int32_t channel)
{
    // lookup the channel
    if(channel == -1) channel= ADC_CHANNEL_VREFINT;
    else if(channel == -2) channel= ADC_CHANNEL_VBAT;
    else if(channel < 0 || channel > 5) return std::numeric_limits<float>::infinity();
    else channel= adc_channel_lut[channel];

    select_channel(channel);
    HAL_ADC_Start(&AdcHandle);
    uint32_t value= ADC3_GetValue();
    HAL_ADC_Stop(&AdcHandle);
    float v = 3.3F * ((float)value / get_max_value());
    return v;
}
