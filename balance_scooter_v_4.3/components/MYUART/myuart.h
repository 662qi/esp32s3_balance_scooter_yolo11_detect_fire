#ifndef __MYUART_H__
#define __MYUART_H__

#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ------ 配置参数，可在此处修改 ------ */
#define UART_NUM        UART_NUM_1          // 所使用的串口号
#define UART_BAUD_RATE  9600                // 串口波特率：9600
#define UART_DATA_BITS  UART_DATA_8_BITS    // 数据位长度
#define UART_STOP_BITS  UART_STOP_BITS_1    // 停止位长度
#define UART_TX_IO      GPIO_NUM_36         // 发送引脚
#define UART_RX_IO      GPIO_NUM_37         // 接收引脚
#define UART_RTX_IO     -1                  // 发送硬件流控引脚
#define UART_CTS_IO     -1                  // 接收硬件流控引脚
#define UART_TX_BUF     1024                // 发送缓冲区大小
#define UART_RX_BUF     1024                // 接收缓冲区大小
#define UART_QUEUE_SIZE 0                   // 串口驱动程序内部缓冲队列大小
#define UART_QUEUE      NULL                // 串口驱动程序内部缓冲队列句柄
#define UART_INTR_FLAGS 0                   // 串口中断标志
#define RX_PACKET_MAX_LEN 100               // 接收数据包的最大长度

/* ------ 函数所需参数声明 ------ */
extern uint8_t myuart_RxFlag;
extern char myuart_RxPacket[RX_PACKET_MAX_LEN];

/* ------ 函数声明 ------ */
void myuart_init(void);
void myuart_send_byte(uint8_t byte);
void myuart_send_string(const char *str);
void myuart_send_array(const uint8_t *arr, uint8_t len);
void myuart_printf(const char *format, ...);
int myuart_receive(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms);
int myart_available(void);
uint8_t myuart_GetRxFlag(void);

#endif
