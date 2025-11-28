#include <stdint.h>
#include "uart.h"

#define GPIOAEN		(1U<<0)
#define GPIODEN		(1U<<3)
#define UART2EN		(1U<<17)
#define UART3EN		(1U<<18)
#define CRCEN		(1U<<12)

#define DBG_UART_BAUDRATE		115200
#define SYS_FREQ				16000000
#define APB1_CLK				SYS_FREQ
#define CR1_TE					(1U<<3)
#define CR1_RE					(1U<<2)
#define CR1_UE					(1U<<13)
#define SR_TXE					(1U<<7)
#define SR_RXNE 				(1U<<5)
#define BL_NACK 0x7F
#define BL_ACK 0xA5

static void uart_set_baudrate(uint32_t periph_clk,uint32_t baudrate);
static void uart_write(int ch);

int __io_putchar(int ch)
{
	uart3_write(ch);
	return ch;
}

void uart_init(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set the mode of PA2 to alternate function mode*/
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |=(1U<<5);
	/*Set the mode of PA3 to alternate function mode*/
	GPIOA->MODER &=~(1U<<6);
	GPIOA->MODER |=(1U<<7);

	/*Set alternate function type to AF7(UART2_TX) PA2*/
	GPIOA->AFR[0] |=(1U<<8);
	GPIOA->AFR[0] |=(1U<<9);
	GPIOA->AFR[0] |=(1U<<10);
	GPIOA->AFR[0] &=~(1U<<11);

	/*Set alternate function type to AF7(UART2_RX) PA3*/
	GPIOA->AFR[0] |=(1U<<12);
	GPIOA->AFR[0] |=(1U<<13);
	GPIOA->AFR[0] |=(1U<<14);
	GPIOA->AFR[0] &=~(1U<<15);

	/*Enable clock access to UART2*/
     RCC->APB1ENR |=	UART2EN;

	/*Configure uart baudrate*/
     uart_set_baudrate(APB1_CLK,DBG_UART_BAUDRATE);

	/*Configure transfer direction*/
     USART2->CR1 = CR1_TE | CR1_RE;

	/*Enable UART Module*/
     USART2->CR1 |= CR1_UE;
}

void uart3_init(void)
{
	RCC->AHB1ENR |= GPIODEN;

	/*Set the mode of PD8 to alternate function mode*/
	GPIOA->MODER &=~(1U<<16);
	GPIOA->MODER |=(1U<<17);


	/*Set alternate function type to AF7(UART3_TX)*/
	GPIOA->AFR[1] |=(1U<<0);
	GPIOA->AFR[1] |=(1U<<1);
	GPIOA->AFR[1] |=(1U<<2);
	GPIOA->AFR[1] &=~(1U<<3);


	/*Enable clock access to UART3*/
     RCC->APB1ENR |=	UART3EN;
}

void uart_write(int ch)
{
	/*Make sure transmit data register is empty*/
	while(!(USART2->SR & SR_TXE)){}

	/*Write to transmit data register*/
	USART2->DR =(ch & 0xFF);
}

void uart3_write(int ch)
{
	/*Make sure transmit data register is empty*/
	while(!(USART3->SR & SR_TXE)){}

	/*Write to transmit data register*/
	USART3->DR =(ch & 0xFF);
}


static uint16_t compute_uart_bd(uint32_t periph_clk,uint32_t baudrate)
{
	return((periph_clk + (baudrate/2U))/baudrate);
}

static void uart_set_baudrate(uint32_t periph_clk,uint32_t baudrate)
{
	USART2->BRR = compute_uart_bd(periph_clk,baudrate);
}

uint8_t uart_read(void)
{
    // Wait until receive data register is not empty
    while(!(USART2->SR & SR_RXNE)){}

    // Read data from receive data register
    return (uint8_t)USART2->DR;
}
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        // Poll until transmit data register is empty
        while (!(USART2->SR & SR_TXE)) {}

        // Write the data byte to the transmit register
        USART2->DR = (pBuffer[i] & 0xFF);
    }
}

void bootloader_send_nack(void)
{
    // Poll until transmit data register is empty
    while (!(USART2->SR & SR_TXE)) {}

    // Write the NACK byte to the transmit register
    USART2->DR = (BL_NACK & 0xFF);
}

void bootloader_send_ack(uint8_t follow_len)
{
    // Send the ACK byte
    while (!(USART2->SR & SR_TXE)) {}
    USART2->DR = (BL_ACK & 0xFF);

    // Send the follow_len byte
    while (!(USART2->SR & SR_TXE)) {}
    USART2->DR = (follow_len & 0xFF);
}
