#include "lcd.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

void lcd_send_cmd (char cmd)
{

	uint8_t i2c_frame_data[4];

	i2c_frame_data[0] = (cmd & 0xF0) | 0x0c;
	i2c_frame_data[1] = i2c_frame_data[0] & 0xFB;

	i2c_frame_data[2] = ((cmd << 4) & 0xF0) | 0x0c;
	i2c_frame_data[3] = i2c_frame_data[2] & 0xFB;

	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, i2c_frame_data, sizeof(i2c_frame_data), TIMEOUT);

	HAL_Delay(1);
}



void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t i2c_frame_data[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	i2c_frame_data[0] = data_u|0x0D;  //en=1, rs=0
	i2c_frame_data[1] = data_u|0x09;  //en=0, rs=0
	i2c_frame_data[2] = data_l|0x0D;  //en=1, rs=0
	i2c_frame_data[3] = data_l|0x09;  //en=0, rs=0

	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, i2c_frame_data, sizeof(i2c_frame_data), TIMEOUT);

	HAL_Delay(1);
}

void lcd_clear (void)  // clear display
{
	lcd_send_cmd(0x01);
	HAL_Delay(1);
}

void lcd_init ()
{
    // 4-bit mode initialisation

	HAL_Delay(50); // wait >40ms
	lcd_send_cmd(0x30);
	HAL_Delay(5); // wait >4.1ms
	lcd_send_cmd(0x30);
	HAL_Delay(1); // wait >100us
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20); // 4bit mode
	HAL_Delay(10);

	 // display initialisation

	lcd_send_cmd(0x28); // function set DL=0 (4bit), N=1 (2 line display), F=0 (5x8 chars)
	HAL_Delay(10);
	lcd_send_cmd(0x08); // display off d,c,b=0
	HAL_Delay(10);
	lcd_clear();
	HAL_Delay(10);
	lcd_send_cmd(0x06); // entry mode set, I/D=1 (inc cursor), S=0 (no shift
	HAL_Delay(10);
	lcd_send_cmd(0x0C); // display on d=1, c,b=0
	HAL_Delay(10);

	HAL_Delay(1);

}

void lcd_send_string (char *str)
{
	while (*str) {
		lcd_send_data((uint8_t)(*str));
		str++;
	}
	HAL_Delay(1);
}

// line_num should be 0 for the 1st line, and 1 for the second line
void lcd_set_line_cursor (int line_num)
{
	switch (line_num) {
		case 0:
			lcd_send_cmd(0x80);
			break;
		case 1:
			lcd_send_cmd(0xC0);
			break;
		default:
	}
}
