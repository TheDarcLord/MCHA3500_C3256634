#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "potentiometer.h"
#include "stm32f4xx_hal_adc.h"

ADC_HandleTypeDef _hadc1;

static uint8_t _is_init = 0;

void pot_init(void)
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
        ADC_ChannelConfTypeDef sConfigADC;
        sConfigADC.Channel = ADC_CHANNEL_8;
        sConfigADC.Rank = 1;
        sConfigADC.SamplingTime = ADC_SAMPLETIME_480CYCLES;
        sConfigADC.Offset = 0;

        /* Enable ADC1 clock */
        /* Enable GPIOB clock */
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure PB0 in analog mode, no pullup */
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.Pin = GPIO_PIN_0;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* Initialise the ADC */ 
        if(HAL_ADC_Init(&_hadc1) != HAL_OK)
        {
            printf("Error initialising ADC! \n");
            return;
        }
        /* Initialise the ADC channel config */
        if(HAL_ADC_ConfigChannel(&_hadc1, &sConfigADC) != HAL_OK)
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

void pot_deinit(void)
{
    HAL_ADC_DeInit(&_hadc1);
    _is_init = 0;
}

uint16_t pot_get_value(void)
{
    /* Poll the ADC conversion */
    uint16_t result = 0;
    if(HAL_ADC_PollForConversion(&_hadc1, 0xFF) != HAL_OK)
    {
        printf("Error polling for ADC conversion! \n");
    }
    else
    {
        /* Get and return the 12-bit result */
        result = HAL_ADC_GetValue(&_hadc1);
    }
    return result;
}
