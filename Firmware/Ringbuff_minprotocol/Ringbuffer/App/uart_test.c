#include "uart_test.h"

#define UART_LEN	64
uint8_t buff_uart_test[UART_LEN];
uint8_t index_uart =0;
uint8_t flag_uart =0;
uint8_t data;
extern UART_HandleTypeDef huart1;
void uart_handle(void)
{
	if(flag_uart)
	{
		HAL_UART_Transmit(&huart1,buff_uart_test,index_uart,100);
		index_uart=0;
		flag_uart =0;
	}
}

void receive_rx_test(uint8_t data_rx)
{
	if( data_rx == '\n')
	{
		flag_uart =1;
	}
	else
	{
		buff_uart_test[index_uart++] = data_rx;
		
	}
}

void uart_test_init(void)
{
	
}
