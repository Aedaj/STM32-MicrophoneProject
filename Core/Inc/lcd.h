#include "stm32f4xx_hal.h"

#define SLAVE_ADDRESS_LCD 0x4e // change this according to your setup
#define TIMEOUT 5000

void lcd_init ();

void lcd_send_cmd (char cmd);

void lcd_send_data (char data);

void lcd_send_string (char *str);

void lcd_clear ();

void lcd_set_line_cursor (int line_num);
