#include "oled.h"

/**
  *@brief	OLED写入一字节命令
  *@param	cmd     命令
  *@retval	无
  */
void oled_write_cmd(uint8_t cmd)
{
    // 定义命令链句柄
    i2c_cmd_handle_t cmd_h = i2c_cmd_link_create();

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);    // 寻址
    i2c_master_write_byte(cmd_h, 0x00, true);           // Control Byte(写命令)
    i2c_master_write_byte(cmd_h, cmd, true);            // 写入命令
    i2c_master_stop(cmd_h);

    i2c_master_cmd_begin(I2C_NUM_0, cmd_h, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd_h);
}

/**
  *@brief	OLED写入一字节数据
  *@param	data    数据
  *@retval	无
  */
void oled_write_data(uint8_t data)
{
    // 定义命令链句柄
    i2c_cmd_handle_t cmd_h = i2c_cmd_link_create();

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);    // 寻址
    i2c_master_write_byte(cmd_h, 0x40, true);           // Control Byte(写数据)
    i2c_master_write_byte(cmd_h, data, true);           // 写入数据
    i2c_master_stop(cmd_h);

    i2c_master_cmd_begin(I2C_NUM_0, cmd_h, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd_h);
}

/**
  *@brief	OLED写入多字节数据
  *@param	data    数据
  *@retval	无
  */
void oled_write_datas(uint8_t *data, uint8_t len)
{
    // 定义命令链句柄
    i2c_cmd_handle_t cmd_h = i2c_cmd_link_create();

    i2c_master_start(cmd_h);
    i2c_master_write_byte(cmd_h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);    // 寻址
    i2c_master_write_byte(cmd_h, 0x40, true);           // Control Byte(写数据)
    i2c_master_write(cmd_h, data, len, true);           // 写入多字节数据
    i2c_master_stop(cmd_h);

    i2c_master_cmd_begin(I2C_NUM_0, cmd_h, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd_h);
}

/**
  *@brief	OLED初始化
  *@param	无
  *@retval	无
  */
void oled_init(void)
{
    myi2c0_init();

    vTaskDelay(pdMS_TO_TICKS(100));

    oled_write_cmd(0xAE);               // 关闭显示（休眠）
    oled_write_cmd(0xD5);               // 设置时钟分频因子
    oled_write_cmd(0x80);               // 分频值 = 1, 振荡频率 = 默认

    oled_write_cmd(0xA8);               // 设置驱动路数（1/64占空比）
    oled_write_cmd(0x3F);               // 0x3F = 64duty，适用于0.96寸OLED

    oled_write_cmd(0xD3);               // 设置显示偏移
    oled_write_cmd(0x00);               // 无偏移

    oled_write_cmd(0x40);               // 设置显示起始行（第0行开始）

    oled_write_cmd(0x8D);               // 电荷泵配置（必须开，否则不亮）
    oled_write_cmd(0x14);               // 开启电荷泵（关键！）

    oled_write_cmd(0x20);               // 设置内存地址模式
    oled_write_cmd(0x00);               // 水平地址模式

    oled_write_cmd(0xA1);               // 段重映射：左右反转
    oled_write_cmd(0xC8);               // COM扫描方向：上下反转

    oled_write_cmd(0xDA);               // 设置COM硬件引脚配置
    oled_write_cmd(0x12);               // 适配0.96寸OLED

    oled_write_cmd(0x81);               // 设置对比度
    oled_write_cmd(0xCF);               // 对比度值（0~255）

    oled_write_cmd(0xD9);               // 设置预充电周期
    oled_write_cmd(0xF1);               // 充电周期配置

    oled_write_cmd(0xDB);               // 设置VCOM取消电平
    oled_write_cmd(0x30);               // 配置为0.83Vcc

    oled_write_cmd(0xA4);               // 全局显示开启，跟随GDDRAM内容

    oled_write_cmd(0xA6);               // 正常显示（1亮0灭）

    oled_write_cmd(0xAF);               // 打开显示（OLED点亮）

    vTaskDelay(pdMS_TO_TICKS(100));
    oled_clear();
}

/**
  *@brief	设置OLED光标位置
  *@param	x       横坐标位置(0 ~ 127)
  *@param	page    页指定(0 ~ 7)
  *@retval	无
  */
void oled_set_cursor(uint8_t x, uint8_t page)
{
    oled_write_cmd(0x00 | (x & 0x0F));          // 横坐标低4位
    oled_write_cmd(0x10 | ((x & 0xF0) >> 4));   // 横坐标高4位
    oled_write_cmd(0xB0 | page);                // 页地址
}

/**
  *@brief	OLED清屏
  *@param	无
  *@retval	无
  */
void oled_clear(void)
{
    /* 单字节清屏函数(太慢)
    for(uint8_t i = 0; i < 8; i++){
        oled_set_cursor(0, i);
        for(uint8_t j = 0; j < 128; j++){
            oled_write_data(0x00);
        }
    }
    */

   // 多字节清屏函数
    uint8_t buf[128] = {0};
    for(uint8_t i=0; i<8; i++)
    {
        oled_set_cursor(0, i);
        oled_write_datas(buf, 128);  // 一次性发128字节
    }
}

/**
  *@brief	OLED显示一个字符
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	ch       所显示的字符
  *@param	fontsize 所显示字符大小
  *@retval	无
  *@note    2026.5.1 单字节发送，速度可能有些慢
  *@note    2026.5.2 已改为多字节发送
  */
void oled_show_char(uint8_t x, uint8_t page, char ch, uint8_t fontsize)
{
    if(fontsize == 6)
    {
        oled_set_cursor(x, page);
        /* 单字节发送(太慢)
        for(uint8_t i = 0; i < 6; i++)
        {
            oled_write_data(OLED_F6x8[ch - ' '][i]);
        }
        */
        // 多字节发送
        oled_write_datas((uint8_t *)OLED_F6x8[ch - ' '], 6);
    }
    else if(fontsize == 8)
    {
        oled_set_cursor(x, page);
        /*
        for(uint8_t i = 0; i < 8; i++)
        {
            oled_write_data(OLED_F8x16[ch - ' '][i]);
        }
        */
        oled_write_datas((uint8_t *)OLED_F8x16[ch - ' '], 8);
        oled_set_cursor(x, page+1);
        /*
        for(uint8_t i = 0; i < 8; i++)
        {
            oled_write_data(OLED_F8x16[ch - ' '][i+8]);
        }
        */
        oled_write_datas((uint8_t *)OLED_F8x16[ch - ' '] + 8, 8);
    }
}

/**
  *@brief	OLED显示字符串
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	*str     所显示的字符串
  *@param	fontsize 所显示字符大小
  *@retval	无
  */
void oled_show_string(uint8_t x, uint8_t page, char *str, uint8_t fontsize)
{
    for(uint8_t i = 0; str[i] != '\0'; i++)
    {
        oled_show_char(x + i * fontsize, page, str[i], fontsize);
    }
}

/**
  *@brief	OLED显示图像
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	width    所显示的图的宽
  *@param	height   所显示的图的高
  *@param	*image   所显示的图
  *@retval	无
  */
void oled_show_image(uint8_t x, uint8_t page, uint8_t width, uint8_t height, const uint8_t *image)
{
    for(uint8_t i = 0; i < height; i++)
    {
        oled_set_cursor(x, page + i);
        for(uint8_t j = 0; j < width; j++)
        {
            oled_write_data(image[j]);
        }
    }
}

/**
  *@brief	OLED显示一串汉字
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	*image   一串汉字
  *@retval	无
  *@note    目前还有一些问题，无法显示汉字(2026.5.1)
  */
void oled_show_chinese(uint8_t x, uint8_t page, char *chinese)
{
    // 放入单个寻找的字的编码
    char sigle_chinese[OLED_CHN_CHAR_WIDTH + 1] = {0};
    uint8_t pChinese = 0;       // 遍历3字节(UTF-8编码格式下)后清零，用于控制结束
    uint8_t pIndex;             // 指定对应的汉字位置

    for(uint8_t i = 0; chinese[i] != '\0'; i++)
    {
        sigle_chinese[pChinese] = chinese[i];
        if(pChinese >= 3)       // 找够3字节(UTF-8编码格式下)数据
        {
            pChinese = 0;

            // 遍历字模库
            for(pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex++)
            {
                if(strcmp(OLED_CF16x16[pIndex].Index, sigle_chinese) == 0)
                {
                    break;
                }
            }

            // 显示汉字
            oled_show_image(x + ((i+1) / 3 - 1) * 16, page, 16, 2, OLED_CF16x16[pIndex].Data);
        }
    }
}

/**
  *@brief	次方
  *@param	X        底数
  *@param	Y        指数
  *@retval	无
  */
uint32_t oled_pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//结果默认为1
	while (Y --)			//累乘Y次
	{
		Result *= X;		//每次把X累乘到结果上
	}
	return Result;
}

/**
  *@brief	OLED显示数字
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	num      数字
  *@param	length   长度
  *@param	fontsize 所显示字符大小
  *@retval	无
  */
void oled_show_num(uint8_t x, uint8_t page, uint32_t num, uint8_t length, uint8_t fontsize)
{
	uint8_t i;
	for (i = 0; i < length; i++)		//遍历数字的每一位							
	{
		/*调用oled_show_num函数，依次显示每个数字*/
		/*num / oled_pow(10, Length - i - 1) % 10 可以十进制提取数字的每一位*/
		/*+ '0' 可将数字转换为字符格式*/
		oled_show_char(x + i * fontsize, page, num / oled_pow(10, length - i - 1) % 10 + '0', fontsize);
	}
}

/**
  *@brief	OLED显示有符号数字
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	num      数字
  *@param	length   长度
  *@param	fontsize 所显示字符大小
  *@retval	无
  */
void oled_show_signed_num(uint8_t x, uint8_t page, int32_t num, uint8_t length, uint8_t fontsize)
{
	uint8_t i;
    uint32_t num1;

    if(num >= 0)
    {
        oled_show_char(x, page, '+', fontsize);
        num1 = num;
    }
    else
    {
        oled_show_char(x, page, '-', fontsize);
        num1 = -num;
    }

	for (i = 0; i < length; i++)		//遍历数字的每一位							
	{
		/*调用oled_show_num函数，依次显示每个数字*/
		/*num / oled_pow(10, Length - i - 1) % 10 可以十进制提取数字的每一位*/
		/*+ '0' 可将数字转换为字符格式*/
		oled_show_char(x + (i+1) * fontsize, page, num1 / oled_pow(10, length - i - 1) % 10 + '0', fontsize);
	}
}

/**
  *@brief	OLED显示浮点数
  *@param	x        横坐标位置(0 ~ 127)
  *@param	page     页指定(0 ~ 7)
  *@param	num      数字
  *@param	ilength  整数部分长度
  *@param	flength  小数部分长度
  *@param	fontsize 所显示字符大小
  *@retval	无
  *@note    2026.5.4 做了OLED显示float
  *@note    2026.5.5 修改了此函数的Bug：Temp会溢出导致整个系统卡死
  */
void oled_show_float(uint8_t x, uint8_t page, float num, uint8_t ilength, uint8_t flength, uint8_t fontsize)
{
    // 修改，删除了Temp，改用分布变量存储
    uint32_t int_part, frac_part;
    float frac_temp;
	
    /* --- 未进行修改，保持不变 --- */
	if (num >= 0)						    //数字大于等于0
	{
		oled_show_char(x, page, '+', fontsize);	//显示+号
	}
	else									//数字小于0
	{
		oled_show_char(x, page, '-', fontsize);	//显示-号
		num = -num;					        //Number取负
	}
    x += fontsize;
	
    /* --- 未进行修改，保持不变 --- */
	/*显示整数部分*/
    int_part = (uint32_t)num;
	oled_show_num(x , page, int_part, ilength, fontsize);
    x += fontsize * ilength;
	
    /* --- 未进行修改，保持不变 --- */
	/*显示小数点*/
	oled_show_char(x , page, '.', fontsize);
    x += fontsize;
	
    /* --- 重点修改部分 --- */
    frac_temp = num - int_part;             // 取出小数部分，存入frac_temp
    for(uint8_t i = 0;  i < flength; i++)
    {
        frac_temp *= 10;
        frac_part = (uint32_t)frac_temp;    // 将一位小数取出到frac_part中进行显示
        oled_show_char(x + i*fontsize, page, frac_part + '0', fontsize);
        frac_temp -= frac_part;
    }
}
