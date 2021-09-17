#include "lcd.h"
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <string.h>

#define LCD_CMD_BIAS     0x52 // 0b1000 0101 0010  1/3duty 4com
#define LCD_CMD_SYSDIS   0x00 // 0b1000 0000 0000  Turn off both system oscillator and LCD bias generator
#define LCD_CMD_SYSEN    0x02 // 0b1000 0000 0010  Turn on system oscillator
#define LCD_CMD_LCDOFF   0x04 // 0b1000 0000 0100  Turn off LCD bias generator
#define LCD_CMD_LCDON    0x06 // 0b1000 0000 0110  Turn on LCD bias generator
#define LCD_CMD_XTAL     0x28 // 0b1000 0010 1000  System clock source, crystal oscillator
#define LCD_CMD_RC256    0x30 // 0b1000 0011 0000  System clock source, on-chip RC oscillator
#define LCD_CMD_TONEON   0x12 // 0b1000 0001 0010  Turn on tone outputs
#define LCD_CMD_TONEOFF  0x10 // 0b1000 0001 0000  Turn off tone outputs
#define LCD_CMD_WDTDIS1  0x0A // 0b1000 0000 1010  Disable WDT time-out flag output

#define LCD_MODE_CMD  0x08
#define LCD_MODE_DATA 0x05

#define LCD_MAX_TRANSFER_SZ (LCD_SYMBOLS + 2)

static uint32_t lcd_spi;
static uint32_t lcd_cs_port;
static uint16_t lcd_cs_gpio;

const uint8_t lcd_digits_lut[10] = {
    LCD_SYMS_0, LCD_SYMS_1, LCD_SYMS_2, LCD_SYMS_3, LCD_SYMS_4,
    LCD_SYMS_5, LCD_SYMS_6, LCD_SYMS_7, LCD_SYMS_8, LCD_SYMS_9,
};

static void lcd_write(uint8_t *data, uint8_t sz)
{
    gpio_clear(lcd_cs_port, lcd_cs_gpio);
    for (int i = 0; i < sz; i++)
    {
        spi_xfer(lcd_spi,data[i]);
    }
    gpio_set(lcd_cs_port, lcd_cs_gpio);
}

static void lcd_send_cmd(uint8_t cmd)
{
    // Prepare the packet
    uint8_t cmd_packet[2];
    cmd_packet[0] = LCD_MODE_CMD << 4;
    cmd_packet[0] |= (cmd >> 4) & 0x0F;
    cmd_packet[1] =  (cmd << 4) & 0xF0;

    lcd_write(cmd_packet, sizeof(cmd_packet));
}

void lcd_send_data(uint8_t *data)
{
    static uint8_t write_buf[LCD_MAX_TRANSFER_SZ];
    static uint8_t repack_buf[LCD_SYMBOLS];

    for (uint8_t pos = 0; pos < LCD_SYMBOLS; pos++)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            uint8_t shift = 4;
            if (i & 0x01)
                shift = 0;
            if (data[pos] & (1 << (7-i)))
                repack_buf[i >> 1] |= (1 << (3-pos)) << shift;
            else
                repack_buf[i >> 1] &= ~((1 << (3-pos)) << shift);
        }
    }

    memset(write_buf, 0, sizeof(write_buf));

    write_buf[0] = LCD_MODE_DATA << 5;
    for (uint8_t i = 0; i < LCD_SYMBOLS; i++)
    {
        write_buf[i + 1]  |= (repack_buf[i] >> 1) & 0x7F;
        write_buf[i + 2]   = (repack_buf[i] << 7) & 0x80;
    }

    lcd_write(write_buf, sizeof(write_buf));
}

void lcd_init(uint32_t spi, uint32_t cs_port, uint16_t cs_gpio)
{
    lcd_spi = spi;
    lcd_cs_port = cs_port;
    lcd_cs_gpio = cs_gpio;
    lcd_send_cmd(LCD_CMD_BIAS);
    lcd_send_cmd(LCD_CMD_RC256);
    lcd_send_cmd(LCD_CMD_SYSDIS);
    lcd_send_cmd(LCD_CMD_WDTDIS1);
    lcd_send_cmd(LCD_CMD_SYSEN);
    lcd_send_cmd(LCD_CMD_LCDON);
}

void lcd_show_rpm(uint32_t rpm)
{
	uint8_t data[4];
	if (rpm > 99999)
	{
		data[0] = LCD_SYMS_MINUS;
		data[1] = LCD_SYMS_MINUS;
		data[2] = LCD_SYMS_MINUS;
		data[3] = LCD_SYMS_MINUS;
		lcd_send_data(data);
		return;
	}

	rpm /= 100;
	data[3] = 0;
	data[2] = lcd_digits_lut[rpm % 10];
	rpm /= 10;
	data[1] = lcd_digits_lut[rpm % 10] | LCD_SEG_DOT;
	rpm /= 10;
	if (rpm)
	{
		data[0] = lcd_digits_lut[rpm];
	}
	else
	{
		data[0] = 0;
	}
	lcd_send_data(data);
}

void lcd_show_time(uint32_t sec)
{
	uint8_t data[4];
	uint32_t mn = sec / 60;
	uint32_t hr = mn / 60;
	mn = mn % 60;

	if (sec > (60*60*100 - 1))
	{
		data[0] = LCD_SYMS_MINUS;
		data[1] = LCD_SYMS_MINUS | LCD_SEG_DOT;
		data[2] = LCD_SYMS_MINUS | LCD_SEG_DOT;
		data[3] = LCD_SYMS_MINUS;
		lcd_send_data(data);
		return;
	}

	data[1] = lcd_digits_lut[hr % 10];
	hr /= 10;
	if (hr)
	{
		data[0] = lcd_digits_lut[hr];
	}
	else
	{
		data[0] = 0;
	}

	data[3] = lcd_digits_lut[mn % 10];
	mn /= 10;
	data[2] = lcd_digits_lut[mn];

	data[2] |= LCD_SEG_DOT;
	data[1] |= LCD_SEG_DOT;

	lcd_send_data(data);
}
