#include "uart_servo.h"

UART_HandleTypeDef _uart3;
GPIO_InitTypeDef _uartGPIO;

void servo_comms_init(void) {

    __USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    _uartGPIO.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    _uartGPIO.Mode = GPIO_MODE_AF_PP;
    _uartGPIO.Alternate = GPIO_AF7_USART3;
    _uartGPIO.Pull = GPIO_PULLUP;
    _uartGPIO.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &_uartGPIO);

    _uart3.Instance = USART3;
    _uart3.Init.BaudRate = 115200;
    _uart3.Init.WordLength = UART_WORDLENGTH_8B;
    _uart3.Init.StopBits = UART_STOPBITS_1;
    _uart3.Init.Parity = UART_PARITY_NONE;
    _uart3.Init.Mode = UART_MODE_TX_RX;
    _uart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    _uart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&_uart3);
}

void servo_command(void) {
    char c1[] = {0xAA, 0x00, 0x00, 0x00};
    char c2[] = {0x84, 0x00, 0x40, 0x3E};

    HAL_UART_Transmit(&_uart3, (uint8_t*)&c1, 4, 0xFFFF);
    HAL_UART_Transmit(&_uart3, (uint8_t*)&c2, 4, 0xFFFF);
}