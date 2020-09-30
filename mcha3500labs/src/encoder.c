#include "encoder.h"

static void _encoder_enable_interrupts(void);
static void _encoder_disable_interrupts(void);

static uint8_t _is_init = 0;
static int32_t _count = 0;

float position = 0;

/* ENCODER ->
 * PC3 -> Yellow, ENCA
 * PC2 -> White, ENCB			
 * Gear Rato: 102.08
 * Counts Per Revolution: 6533
*/


/*	Interrupt lines
	In section one (GPIOs) we have 16 interrupt lines. 
	They are line0 to line15 and they also represent pin number. 
	This means, PA0 is connected to Line0 and PA13 is connected to Line13.
*/
void encoder_init(void)
{
    // Set default count
    _count = 0;

    if (!_is_init)  // Guards against multiple initialisations
    {
		__HAL_RCC_GPIOC_CLK_ENABLE();							// Enable GPIOC clock
    	GPIO_InitTypeDef GPIO_InitStruct;
    	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;			// Configure and initialise PC2 and PC3
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;		// External interrupts on the rising and falling edges
		GPIO_InitStruct.Pull = GPIO_NOPULL;						// Pull-up resistors enabled
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);					// Initialise GPIOC

		/* Set the priorities of the EXTI0 and EXTI1 interrupts
		 * Enable the EXTI0 and EXTI1 IRQs using the NVIC
		 * Set the priorities of the EXTI0 and EXTI1 interrupts to
		 * - Preempt: 0x02,
		 * - Sub: 0x00
		*/
		HAL_NVIC_SetPriority(EXTI2_IRQn, 0x0F, 0x0F);
		HAL_NVIC_SetPriority(EXTI3_IRQn, 0x0F, 0x0F);

		_encoder_enable_interrupts();
        _is_init = 1;
    }
}

void encoder_edge_A_isr(void)
{
    // Implement A edge logic to increment or decrement _count
	int A = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	int B = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

	if(A == B) {
		_count++;
	} else {
		_count--;
	}
}

void encoder_edge_B_isr(void)
{
    // Implement B edge logic to increment or decrement _count
	int A = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	int B = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

	if(A == B) {
		_count--;
	} else {
		_count++;
	}
}

void encoder_set_count(int32_t count)
{
    // Atomically set _count
    _encoder_disable_interrupts();
    _count = count;
    _encoder_enable_interrupts();
}

float encoder_get_count(void)
{
    // Automically read _count
    _encoder_disable_interrupts();
    uint32_t count = _count;
    _encoder_enable_interrupts();
    return countToRadians(count);
}

float encoder_pop_count(void)
{
    // Automically read and reset _count
    _encoder_disable_interrupts();
    int32_t count = _count;
	_count = 0;
    _encoder_enable_interrupts();
	return(count);
    
	//return countToRadians(count);
}

void _encoder_enable_interrupts(void)
{
    /* Enable the EXTI0 and EXTI1 IRQs using the NVIC */
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void _encoder_disable_interrupts(void)
{
    /* Disable the EXTI0 and EXTI1 IRQs using the NVIC */
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
}

void EXTI3_IRQHandler(void)
{
    /* Call the encoder edge A isr */
	encoder_edge_A_isr();
    /* Call the HAL GPIO EXTI IRQ Handler and specify the GPIO pin */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI2_IRQHandler(void)
{
    /* Call the encoder edge B isr */
	encoder_edge_B_isr();
    /* Call the HAL GPIO EXTI IRQ Handler and specify the GPIO pin */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

float countToRadians(int32_t c) {
	position = (2 * M_PI * c/CPR) * (180 / M_PI);
	return(position);
}