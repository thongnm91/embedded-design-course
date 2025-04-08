#include <stdio.h>
#include <stddef.h>
#include "stm32l1xx.h"

void clear_buffer(char *b);
void USART1_IRQHandler(void);
void USART1_write(char data);
void USART2_write(char data);
void USART1_Init(void);
void USART2_Init(void);
void timer11_init(void);
void wrong_slave_address(void);
char USART1_read();
char USART2_read();
unsigned short int  CRC16(char *nData,unsigned short int wLength);
void read_7_bytes_from_usartx(char *received_frame);
int read_sensor(int input_address);
void respond_frame(int sensor_value);
void write_debug_msg(char *str, int maxchars);
void write_debug_frame(char *str, int maxchars);
void LED_Init();
void blink_led();

void delay_us(unsigned long delay);
void delay_Us(int delay);
void SetSysClock(void);
void delay_Ms(int delay);

void read_dht22_humidity_and_temperature(uint16_t* temp,uint16_t* hum,uint16_t*  c_sum,signed char* minus);
