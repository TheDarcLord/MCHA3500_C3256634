#include "motor.h"

static TIM_HandleTypeDef   _htim3;
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
 *   PWM      PC7   - Pulse width modulation input: PWM signal
 *   INA      PC2   - Motor direction input A (“clockwise” input).
 *   INB      PC3   - Motor direction input B (“counterclockwise” input).
 *   CS             - Current sense output
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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Initialise PWM pin */
    
    GPIO_InitStructure.Pin      = GPIO_PIN_7;
    GPIO_InitStructure.Mode     = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull     = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure Timer4 to generate single PWM output on GPIOC7 */
    /* Set timer, prescaler, mode, period and clkdiv */
    _htim3.Instance = TIM3;
    _htim3.Init.Prescaler = 0x0000;
    _htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim3.Init.Period = 0xFFFF;
    _htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    
    _sConfigPWM.OCMode = TIM_OCMODE_PWM2;
    _sConfigPWM.OCIdleState = TIM_OCIDLESTATE_SET;
    _sConfigPWM.Pulse = 0x0000;     // Value Used to edit PWM?
    _sConfigPWM.OCPolarity = TIM_OCPOLARITY_LOW;
    _sConfigPWM.OCFastMode = TIM_OCFAST_ENABLE;

    HAL_TIM_PWM_Init(&_htim3);
    HAL_TIM_PWM_ConfigChannel(&_htim3, &_sConfigPWM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&_htim3, TIM_CHANNEL_2);
}

void _motor_gpio_init(void) { 
    /* Enable peripheral clocks */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* Initialise DIRECTION pins */
    //GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin      = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStructure.Mode     = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull     = GPIO_PULLUP;
    GPIO_InitStructure.Speed    = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    _motor_set_direction(MOTOR_DIR_BRK);
}

void _motor_set_direction(uint8_t dir)
{
    if(dir == MOTOR_DIR_FWD) {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
    } else if(dir == MOTOR_DIR_BCK) {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
    }
}

uint16_t _motor_set_dutycyle(float percent) {
    if(percent < 0) {
        percent = percent * -1;
        _motor_set_direction(MOTOR_DIR_BCK);
    } else if(percent > 0) {
        _motor_set_direction(MOTOR_DIR_FWD);
    } else {
        _motor_set_direction(MOTOR_DIR_BRK);
    }
    
    uint16_t dutyCycle = percent * MAXDUTY;
    __HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_2, dutyCycle);
    return(dutyCycle);
}

void motor_set(float voltage) {
    float p = voltage / MAXVOLT;

    if(p < 0) {
        p = p * -1;
        _motor_set_direction(MOTOR_DIR_BCK);
    } else if(p > 0) {
        _motor_set_direction(MOTOR_DIR_FWD);
    } else {
        _motor_set_direction(MOTOR_DIR_BRK);
    }
    uint16_t dutyCycle = p * MAXDUTY;
    __HAL_TIM_SET_COMPARE(&_htim3, TIM_CHANNEL_2, dutyCycle);
}