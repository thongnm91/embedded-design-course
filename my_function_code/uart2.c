/*
 * uart2.c
 *
 *  Created on: 29.5.2025
 *      Author: thong
 */


static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2))/BaudRate);
}
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk,BaudRate);
}
void uart2_tx_init()
{
	//Enable clock access to gpioA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//Set gpio PA2 mode alternate function
	GPIOA->MODER |= GPIO_MODER_MODER2_1;
	//Set gpio PA2 type UART_TX (AF07)
	GPIOA->AFR[0] |= 0x7U<<GPIO_AFRL_AFRL2_Pos; //0:AFRL 1:AFRH
	//Set gpio PA3 mode alternate function
	GPIOA->MODER |= GPIO_MODER_MODER3_1;
	//Set gpio PA2 type UART_RX (AF07)
	GPIOA->AFR[0] |= 0x7U<<GPIO_AFRL_AFRL3_Pos; //0:AFRL 1:AFRH

	//Enable clock access uart2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //STM32L152_reference_manual.pdf p158/911
	//Configure baudrate
	uart_set_baudrate(USART2,SYS_FREQ,UART_BAUDRATE);

	//Config transfer direction
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE; //TE:Transmit Enable, RE:Receive Enable


	//Enable uart module
	USART2->CR1 |= USART_CR1_UE; //UE: UART Enable

}
void uart2_write(unsigned char ch)
{
	//Make sure the transmit data register is empty
	while(!(USART2->SR & USART_SR_TXE)){}

	//Write to transmit data register
	USART2->DR = ch;
}
void uart2_write_string(char* ch){
	uint8_t i=0;
	while(ch[i]!='\0'){
		uart2_write(ch[i]);
		i++;
	}
	i=0;
}
char uart2_read(){
	//When data is received
	while(!(USART2->SR & USART_SR_RXNE)) {}

	//Read data
	return USART2->DR;
}
