#include "motor.h"

static TIM_HandleTypeDef   _htim1;
static TIM_OC_InitTypeDef  _sConfigPWM;
static GPIO_InitTypeDef  GPIO_InitStructure;

static void _motor_pwm_init(void);
static void _motor_gpio_init(void);

//static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

/* INTERFACE: VNH5019
 *  PINS:   BOARD:  DESCRIPTION;
 *   OUTA     M+    - Output of half-bridge A (motor +)
 *   OUTB     M-    - Output of half-bridge B (motor -)
 *   PWM      PA8   - Pulse width modulation input: PWM signal
 *   INA      PB5   - Motor direction input A (“clockwise” input).
 *   INB      PB4   - Motor direction input B (“counterclockwise” input).
 *   CS       PC4   - Current sense output
 *                    The pin voltage is roughly 140 mV per amp of output
 *                    current when the CS_DIS pin is low or disconnected.
 *   CS_DIS         - Disables the current sense output, CS, when high
*/

void motor_init(void)
{
    if(_is_init == 0) {
        _motor_gpio_init();
        _motor_pwm_init();
        _is_init = 1;
    }
}

void _motor_pwm_init(void) {

     /* Enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Initialise PWM pin -> PA8 TIM1/CH1 */
    
    GPIO_InitStructure.Pin      = GPIO_PIN_8;
    GPIO_InitStructure.Mode     = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull     = GPIO_NOPULL;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure Timer1 to generate single PWM output on GPIOA8 */
    /* Set timer, prescaler, mode, period and clkdiv */
    _htim1.Instance = TIM1;
    _htim1.Init.Prescaler = 0x0000;
    _htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim1.Init.Period = MAXDUTY;
    _htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    
    _sConfigPWM.OCMode = TIM_OCMODE_PWM1;
    _sConfigPWM.Pulse = 0;     // Value Used to edit PWM?
    _sConfigPWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    _sConfigPWM.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_Init(&_htim1);
    HAL_TIM_PWM_ConfigChannel(&_htim1, &_sConfigPWM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&_htim1, TIM_CHANNEL_1);
}

void _motor_gpio_init(void) { 
    /* Enable peripheral clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Initialise DIRECTION pins */
    //GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin      = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStructure.Mode     = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull     = GPIO_PULLUP;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    _motor_set_direction(BRK);
}

void _motor_set_direction(uint8_t dir)
{
    switch(dir) {
        case FWD: // Forward
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
        break;
        case BCK: // Backward
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
        break;
        case BRK: // Brake
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
        break;
    }
}

uint16_t motor_set_voltage(float voltage) {
    /* Voltage = %DC * MAXVOLTAGE
     * DC = Voltage/MAXVOLTAGE
     */
    float percent = fabs(voltage / MAXVOLT);
    set_direction(voltage);
    if(voltage > 0) {
        _motor_set_direction(FWD);
    } else if(voltage < 0) {
        _motor_set_direction(BCK);

    } else {
        _motor_set_direction(BRK);
    }
    uint16_t dutyCycle = percent * MAXDUTY;
    __HAL_TIM_SET_COMPARE(&_htim1, TIM_CHANNEL_1, dutyCycle);
    return(dutyCycle);
}