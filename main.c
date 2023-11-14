#include "stm32f401xc.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_adc.h"

void GPIO_config (void)
{
    // LED pin PC13
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);

    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    // PA1
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);
    // PA2
    //LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
    //LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);
}

void SysTick_config (void)
{
    LL_SYSTICK_SetClkSource (LL_SYSTICK_CLKSOURCE_HCLK_DIV8); // HCLK frequency is set to 84 MHz
    LL_InitTick (10500000, 5250000); // 2 interrupts per second
}

void MCO_config (void) //SYSCLK/2 on PA8
{
    LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed (GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_8_15 (GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_0);
    LL_RCC_ConfigMCO (LL_RCC_MCO1SOURCE_PLLCLK, LL_RCC_MCO1_DIV_2); //only MCO1 is available on this MCU
}

void RCC_config (void)
{
    LL_RCC_DeInit ();
    LL_FLASH_SetLatency (LL_FLASH_LATENCY_2);
    #if defined (MCO)
        MCO_config ();
    #endif //MCO

    /* Enable HSE and wait for activation*/
    LL_RCC_HSE_Enable ();
    while (LL_RCC_HSE_IsReady () != 1);

    LL_RCC_PLL_Disable ();
    LL_RCC_PLL_ConfigDomain_SYS (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_4); //336 is PLLN
    //LL_RCC_PLL_ConfigDomain_48M (LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);
    LL_RCC_PLL_Enable ();
    while (LL_RCC_PLL_IsReady () != 1);

    LL_RCC_SetAHBPrescaler (LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler (LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler (LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource (LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource () != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    //while (LL_RCC_GetUSBClockFreq (LL_RCC_USB_CLKSOURCE) == LL_RCC_PERIPH_FREQUENCY_NO);
}

void ADC_config (void)
{
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_ADC1);
    LL_ADC_SetCommonClock (__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV6);
    
    // ADC calibration
    if (LL_ADC_IsEnabled (ADC1)) 
    {
        LL_ADC_Disable (ADC1);
    }
    while (LL_ADC_IsEnabled (ADC1));
    LL_ADC_StartCalibration (ADC1);
    while (LL_ADC_IsCalibrationOnGoing (ADC1));

    // ADC setup
    LL_ADC_Enable (ADC1);
    LL_ADC_SetResolution (ADC1, LL_ADC_RESOLUTION_12B);  // 12 bit resolution
    LL_ADC_SetDataAlignment (ADC1, LL_ADC_DATA_ALIGN_RIGHT); // fill by zeors on the left
    LL_ADC_SetChannelSamplingTime (ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_3CYCLES); // select the shortest assuming that voltage source has low impedance
    LL_ADC_INJ_SetTriggerSource (ADC1, LL_ADC_INJ_TRIG_SOFTWARE); // software start of conversion
    LL_ADC_INJ_SetTrigAuto (ADC1, LL_ADC_INJ_TRIG_INDEPENDENT);
    LL_ADC_INJ_SetSequencerLength (ADC1, LL_ADC_INJ_SEQ_SCAN_DISABLE); // only one channel is in use
    LL_ADC_INJ_SetSequencerRanks (ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_1); // channel 1 (PA1) is in use
}

size_t cnt = 0;
void SysTick_Handler (void)
{
    if (cnt >= 1)
    {
        LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_13);
        cnt = 0;
    }
    else
    {
        LL_GPIO_SetOutputPin (GPIOC, LL_GPIO_PIN_13);
        cnt++;
    }

    while (LL_ADC_IsActiveFlag_JEOS (ADC1) == 0);
    uint16_t ADC_value = LL_ADC_INJ_ReadConversionData12 (ADC1, LL_ADC_INJ_RANK_1);

    LL_ADC_INJ_StartConversionSWStart (ADC1);
}

int main (void)
{
    RCC_config ();
    GPIO_config ();
    ADC_config ();
    SysTick_config ();

    LL_SYSTICK_EnableIT ();
    
    LL_ADC_INJ_StartConversionSWStart (ADC1); // start the first conversion

    while (1) 
    {
        
    }
}
