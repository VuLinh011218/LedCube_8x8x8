#ifndef UART_TEST_H
#define UART_TEST_H
#include "stm32f1xx_hal.h"
void uart_handle(void);
void receive_rx_test(uint8_t data_rx);
void uart_test_init(void);
#endif
