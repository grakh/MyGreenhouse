#ifndef _LCD_2_H_
#define _LCD_2_H_

#include <avr/io.h>
#include <util/delay.h>

#define PORT_LCD PORTD	
#define DDR_LCD DDRD

#define PORT_CMD PORTB	
#define DDR_CMD DDRB

#define sbi(sfr, bit) (sfr|=(1<<bit))
#define cbi(sfr, bit) (sfr&=~(1<<bit))

#define EN 1
#define RS 0
#define RW 2

/* установка лог. "1" на линии E */
#define LCD_E_SET    sbi(PORT_CMD, EN);
/* установка лог. "0" на линии E */
#define LCD_E_CLR    cbi(PORT_CMD, EN);
/* установка лог. "1" на линии RS */
#define LCD_RS_SET   sbi(PORT_CMD, RS);
/* установка лог. "0" на линии RS */
#define LCD_RS_CLR   cbi(PORT_CMD, RS);

#define LCD_PIN_D4 PD4
#define LCD_PIN_D5 PD5
#define LCD_PIN_D6 PD6
#define LCD_PIN_D7 PD7

/* макрос, указывающий функции, что
передаются команды */
#define LCD_COMMAND  0
/* макрос, указывающий функции, что
передаются данные */
#define LCD_DATA     1

#define lcd_putc(x)  lcd_putbyte(x, LCD_DATA)

/* инициализация портов,
подключенных к жки */
void init_port()
{
	PORT_LCD&=~((1<<LCD_PIN_D7)|(1<<LCD_PIN_D6)|(1<<LCD_PIN_D5)|(1<<LCD_PIN_D4));
	DDR_LCD|=(1<<LCD_PIN_D7)|(1<<LCD_PIN_D6)|(1<<LCD_PIN_D5)|(1<<LCD_PIN_D4);
	PORT_CMD&=~((1<<EN)|(1<<RS)|(1<<RW));
	DDR_CMD|=(1<<EN)|(1<<RS)|(1<<RW);
	_delay_ms(50); //Ждем готовности
}

/* функция передачи тетрады в жки */
void lcd_putnibble(char t)
{
    t<<=4;
    LCD_E_SET;
    _delay_us(50);
    PORT_LCD&=0x0F;
    PORT_LCD|=t;
    LCD_E_CLR;
    _delay_us(50);
}

/* функция передачи байта в жки.
char c - сам байт
char rs - переменная, указывающая что передается:
     rs = 0 - команда (устанавливается линия RS)
	 rs = 1 - данные (сбрасывается линия RS) */
void lcd_putbyte(char c, char rs){
    char highc=0;
    highc=c>>4;
if ((rs==LCD_COMMAND)) {LCD_RS_CLR;} else {LCD_RS_SET;}
    lcd_putnibble(highc);
    lcd_putnibble(c);
}

void lcd_putchar(char c)
{
    char highc=0;
    highc=c>>4;
	LCD_RS_SET;
    lcd_putnibble(highc);
    lcd_putnibble(c);
}

/* функция инициализации работы жки
в 4-битном режиме, без курсора */
void lcd_init()
{
	init_port();
    _delay_ms(15);
    lcd_putnibble(0b00000011);
    _delay_ms(4);
    lcd_putnibble(0b00000011);
    _delay_us(100);
    lcd_putnibble(0b00000011);
	 _delay_ms(1);
    lcd_putnibble(0b00000010);
	_delay_ms(1);
    lcd_putbyte(0x28, LCD_COMMAND); // стоит (0x28) 5x8 0b00101000 меняем (0x2C) 5x10 0b00101100 
    _delay_ms(1);
    lcd_putbyte(0x0C, LCD_COMMAND);
    _delay_ms(1);
    lcd_putbyte(0x06, LCD_COMMAND);
    _delay_ms(1);
	sbi(PORT_CMD, RW);
}

/* функция очистки дисплея и возврата
курсора в начальную позицию*/
void lcd_clear()
{
    lcd_putbyte(0x01, LCD_COMMAND);
    _delay_us(1500);
}

void lcd_light(char on)
{
	if (on==1) sbi(PORT_CMD, RW); else cbi(PORT_CMD, RW);
    _delay_ms(1);
}

/* функция перемещения курсора в заданную позицию
col - номер знакоместа по горизонтальной оси (от 0 до 15)
row - номер строки (0 или 1) */
void lcd_gotoxy(char col, char row)
{
  char adr;
  adr=0x40*row+col;
  adr|=0x80;
  lcd_putbyte(adr, LCD_COMMAND);
}

void lcd_putstring (char stroka[])
{  char i;
   for(i=0;stroka[i]!='\0';i++)
   lcd_putchar(stroka[i]);
}	

 // Функция записи символа в контроллер дисплея 
 void define_char(unsigned char Simvol[],unsigned char addres) 
 {
char i;
// команда 0b01000000 + addres*8 присваение счетчику АС адреса в области CGRAM
lcd_putbyte(0x40+addres*8, LCD_COMMAND);
 
for(i=0;i<8;i++)
lcd_putbyte(Simvol[i],LCD_DATA);
}

#endif;