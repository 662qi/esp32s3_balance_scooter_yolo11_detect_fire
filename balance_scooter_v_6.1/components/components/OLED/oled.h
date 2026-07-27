#ifndef __OLED_H__
#define __OLED_H__

// ========== 总开关宏 ==========
// #define ENABLE_OLED_DISPLAY 1

#ifdef ENABLE_OLED_DISPLAY

#include "myi2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled_data.h"
#include <string.h>

#define OLED_ADDR           0x3C    // OLED地址
#define OLED_Size_F8x16     8
#define OLED_Size_F6x8      6

void oled_write_cmd(uint8_t cmd);
void oled_write_data(uint8_t data);
void oled_write_datas(uint8_t *data, uint8_t len);
void oled_init(void);
void oled_set_cursor(uint8_t x, uint8_t page);
void oled_clear(void);
void oled_show_char(uint8_t x, uint8_t page, char ch, uint8_t fontsize);
void oled_show_string(uint8_t x, uint8_t page, char *str, uint8_t fontsize);
void oled_show_image(uint8_t x, uint8_t page, uint8_t width, uint8_t height, const uint8_t *image);
void oled_show_chinese(uint8_t x, uint8_t page, char *chinese);
void oled_show_num(uint8_t x, uint8_t page, uint32_t num, uint8_t length, uint8_t fontsize);
void oled_show_signed_num(uint8_t x, uint8_t page, int32_t num, uint8_t length, uint8_t fontsize);
void oled_show_float(uint8_t x, uint8_t page, float num, uint8_t ilength, uint8_t flength, uint8_t fontsize);

#endif

#endif
