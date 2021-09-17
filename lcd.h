#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_SEG_DOT  0x01
#define LCD_SEG_G    0x02
#define LCD_SEG_F    0x04
#define LCD_SEG_E    0x08
#define LCD_SEG_D    0x10
#define LCD_SEG_C    0x20
#define LCD_SEG_B    0x40
#define LCD_SEG_A    0x80

#define LCD_SYMBOLS  4

#define LCD_SYMS_0  (LCD_SEG_A | LCD_SEG_B | LCD_SEG_C | LCD_SEG_D | LCD_SEG_E | LCD_SEG_F)
#define LCD_SYMS_1  (LCD_SEG_B | LCD_SEG_C)
#define LCD_SYMS_2  (LCD_SEG_A | LCD_SEG_B | LCD_SEG_D | LCD_SEG_E | LCD_SEG_G)
#define LCD_SYMS_3  (LCD_SEG_A | LCD_SEG_B | LCD_SEG_C | LCD_SEG_D | LCD_SEG_G)
#define LCD_SYMS_4  (LCD_SEG_B | LCD_SEG_C | LCD_SEG_F | LCD_SEG_G)
#define LCD_SYMS_5  (LCD_SEG_A | LCD_SEG_C | LCD_SEG_D | LCD_SEG_F | LCD_SEG_G)
#define LCD_SYMS_6  (LCD_SEG_A | LCD_SEG_C | LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G)
#define LCD_SYMS_7  (LCD_SEG_A | LCD_SEG_B | LCD_SEG_C)
#define LCD_SYMS_8  (LCD_SEG_A | LCD_SEG_B | LCD_SEG_C | LCD_SEG_D | LCD_SEG_E | LCD_SEG_F | LCD_SEG_G)
#define LCD_SYMS_9  (LCD_SEG_A | LCD_SEG_B | LCD_SEG_C | LCD_SEG_D | LCD_SEG_F | LCD_SEG_G)

#define LCD_SYMS_MINUS  LCD_SEG_G

extern const uint8_t lcd_digits_lut[10];

void lcd_init(uint32_t spi, uint32_t cs_port, uint16_t cs_gpio);
void lcd_send_data(uint8_t *data);
void lcd_show_rpm(uint32_t rpm);
void lcd_show_time(uint32_t sec);

#endif /* LCD_H */
