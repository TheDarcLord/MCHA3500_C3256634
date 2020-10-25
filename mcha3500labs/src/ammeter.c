#include "ammeter.h"

ADC_HandleTypeDef _hadc1;

static uint8_t _is_init = 0;

void ammeter_init(void)
{
    if (!_is_init)
    {
        /* Configure ADC1 instance to be initialised with:
                div2 prescaler,
                12-bit resolution, 
                right data aligned, 
                continuous conversion mode,
                one conversion.
        Note:   Config parameters of the ADC_InitTypeDef not mentioned above
                are not required in this lab            */
        _hadc1.Instance = ADC1;
        _hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
        _hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        _hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        _hadc1.Init.ContinuousConvMode = ENABLE;
        _hadc1.Init.NbrOfConversion = 1;

        /* Configure the ADC to:
                use channel 8,
                sequence rank of 1,
                480 cycle sample time,
                offset of 0.                            */
        ADC_ChannelConfTypeDef channelConfigADC;
        channelConfigADC.Channel = ADC_CHANNEL_12;
        channelConfigADC.Rank = 1;
        channelConfigADC.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        channelConfigADC.Offset = 0;

        /* Enable ADC1 clock */
        /* Enable GPIOB clock */
        __HAL_RCC_ADC2_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* Configure PB0 in analog mode, no pullup */
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.Pin = GPIO_PIN_4;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

        /* Initialise the ADC */ 
        if(HAL_ADC_Init(&_hadc1) != HAL_OK)
        {
            printf("Error initialising ADC! \n");
            return;
        }
        /* Initialise the ADC channel config */
        if(HAL_ADC_ConfigChannel(&_hadc1, &channelConfigADC) != HAL_OK)
        {
            printf("Error configuring ADC channel!\n");
            return;
        }
        /* Start the ADC */
        if(HAL_ADC_Start(&_hadc1) != HAL_OK)
        {
            printf("Error starting ADC! \n");
            return;
        }

        _is_init = 1;
    }
}

void ammeter_deinit(void)
{
    HAL_ADC_DeInit(&_hadc1);
    _is_init = 0;
}

double ammeter_get_value(void)
{
    /* Poll the ADC conversion */
    uint16_t result = 0;
    if(HAL_ADC_PollForConversion(&_hadc1, 0xFF) != HAL_OK) {
        printf("Error polling for ADC conversion! \n");
    } else {
        /* Get and return the 12-bit result */
        result = HAL_ADC_GetValue(&_hadc1);
    }

    return ((((double) result) * PIN_VMAX) / (MAX12BIT * V_PER_C)) ;
}
