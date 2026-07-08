#include "myuart.h"

uint8_t myuart_RxFlag = 0;
char myuart_RxPacket[RX_PACKET_MAX_LEN] = {0};

/**
  *@brief	UART接收任务：持续监听UART数据并处理
  *@param	无
  *@retval	无
  */
static void myuart_rx_task(void *arg)
{
    uint8_t rx_state = 0;           // 0:等待包头, 1:接收数据
    uint8_t p_rx = 0;               // 接收数据指针
    uint8_t rx_byte = 0;            // 接收的单个字节

    while(1)
    {
        if(uart_read_bytes(UART_NUM, &rx_byte, 1, 0) == 1)
        {
            switch(rx_state)
            {
                case 0:
                    if(rx_byte == '[')  // 包头
                    {
                        rx_state = 1;
                        p_rx = 0;
                        memset(myuart_RxPacket, 0, RX_PACKET_MAX_LEN);
                    }
                    break;

                case 1:
                    if(rx_byte == ']')  // 包尾
                    {
                        myuart_RxFlag = 1;  // 设置接收完成标志
                        rx_state = 0;       // 回到等待包头状态
                    }
                    else if(p_rx < RX_PACKET_MAX_LEN - 1)  // 防止缓冲区溢出
                    {
                        myuart_RxPacket[p_rx++] = rx_byte;  // 存储接收到的字节
                    }
            
            }
        }
        vTaskDelay(1);      // 避免任务占用过多CPU资源
    }
}

/**
  *@brief	UART初始化
  *@param	无
  *@retval	无
  */
void myuart_init(void)
{
    // 配置时钟源与UART核心参数
    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUD_RATE,            // 波特率
        .data_bits = UART_DATA_BITS,            // 数据位

        .flags.allow_pd = 0,                    // 是否允许系统进入睡眠时，把UART所在的电源域关掉：否
        .flags.backup_before_sleep = 0,         // 与allow_pd效果一致，为旧名称，已作废

        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // 硬件流控模式：不使用
        .parity = UART_PARITY_DISABLE,          // 配置校验模式(校验位)：不开启校验
        .rx_flow_ctrl_thresh = 0,               // 硬件流控阈值(关闭时无效，可任意填写)
        .source_clk = UART_SCLK_DEFAULT,        // 时钟源：默认(APB时钟源)
        .stop_bits = UART_STOP_BITS,            // 停止位
    };
    uart_param_config(UART_NUM, &uart_cfg);

    // 配置输入输出引脚
    uart_set_pin(UART_NUM, UART_TX_IO, UART_RX_IO, UART_RTX_IO, UART_CTS_IO);

    // 配置发送与接收缓冲区
    uart_driver_install(UART_NUM, UART_RX_BUF, UART_TX_BUF, UART_QUEUE_SIZE, UART_QUEUE, UART_INTR_FLAGS);

    // 创建UART接收任务
    xTaskCreate(myuart_rx_task, "myuart_rx_task", 2048, NULL, 10, NULL);
}

/**
  *@brief	UART获取接收完成标志：如果有新数据包接收完成，返回1并清除标志；否则返回0
  *@param	无
  *@retval	1:有新数据包接收完成；0:没有新数据包
  */
uint8_t myuart_GetRxFlag(void)
{
    if(myuart_RxFlag)
    {
        myuart_RxFlag = 0;  // 读取后清除标志
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  *@brief	UART发送单个字节
  *@param	byte        要发送的字节
  *@retval	无
  */
void myuart_send_byte(uint8_t byte)
{
    uart_write_bytes(UART_NUM, (const char*)&byte, 1);
}

/**
  *@brief	UART发送字符串
  *@param	str         字符串指针
  *@retval	无
  */
void myuart_send_string(const char *str)
{
    uart_write_bytes(UART_NUM, str, strlen(str));
}

/**
  *@brief	UART发送数组
  *@param	arr         字节数组指针
  *@param	len         要发送的长度
  *@retval	无
  */
void myuart_send_array(const uint8_t *arr, uint8_t len)
{
    uart_write_bytes(UART_NUM, arr, len);
}

/**
  *@brief	UART格式化打印（类似printf）
  *@param	format      格式化字符串
  *@retval	无
  */
void myuart_printf(const char *format, ...)
{
    char buf[UART_TX_BUF];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    myuart_send_string(buf);
}
/**
  *@brief	UART接收数据（阻塞式）
  *@param	buf         接收缓冲区
  *@param	max_len     最大接收长度
  *@param	timeout_ms  超时时间（毫秒）
  *@retval	实际接收长度
  */
int myuart_receive(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms)
{
    return uart_read_bytes(UART_NUM, buf, max_len, pdMS_TO_TICKS(timeout_ms));
}

/**
  *@brief	UART检查是否有可读数据
  *@param	无
  *@retval	len         数据长度
  */
int myart_available(void)
{
    size_t len = 0;
    uart_get_buffered_data_len(UART_NUM, &len);
    return len;
}
