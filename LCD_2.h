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

/* ��������� ���. "1" �� ����� E */
#define LCD_E_SET    sbi(PORT_CMD, EN);
/* ��������� ���. "0" �� ����� E */
#define LCD_E_CLR    cbi(PORT_CMD, EN);
/* ��������� ���. "1" �� ����� RS */
#define LCD_RS_SET   sbi(PORT_CMD, RS);
/* ��������� ���. "0" �� ����� RS */
#define LCD_RS_CLR   cbi(PORT_CMD, RS);

#define LCD_PIN_D4 PD4
#define LCD_PIN_D5 PD5
#define LCD_PIN_D6 PD6
#define LCD_PIN_D7 PD7

/* ������, ����������� �������, ���
���������� ������� */
#define LCD_COMMAND  0
/* ������, ����������� �������, ���
���������� ������ */
#define LCD_DATA     1

#define lcd_putc(x)  lcd_putbyte(x, LCD_DATA)

/* ������������� ������,
������������ � ��� */
void init_port()
{
	PORT_LCD&=~((1<<LCD_PIN_D7)|(1<<LCD_PIN_D6)|(1<<LCD_PIN_D5)|(1<<LCD_PIN_D4));
	DDR_LCD|=(1<<LCD_PIN_D7)|(1<<LCD_PIN_D6)|(1<<LCD_PIN_D5)|(1<<LCD_PIN_D4);
	PORT_CMD&=~((1<<EN)|(1<<RS)|(1<<RW));
	DDR_CMD|=(1<<EN)|(1<<RS)|(1<<RW);
	_delay_ms(50); //���� ����������
}

/* ������� �������� ������� � ��� */
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

/* ������� �������� ����� � ���.
char c - ��� ����
char rs - ����������, ����������� ��� ����������:
     rs = 0 - ������� (��������������� ����� RS)
	 rs = 1 - ������ (������������ ����� RS) */
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

/* ������� ������������� ������ ���
� 4-������ ������, ��� ������� */
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
    lcd_putbyte(0x28, LCD_COMMAND); // ����� (0x28) 5x8 0b00101000 ������ (0x2C) 5x10 0b00101100 
    _delay_ms(1);
    lcd_putbyte(0x0C, LCD_COMMAND);
    _delay_ms(1);
    lcd_putbyte(0x06, LCD_COMMAND);
    _delay_ms(1);
	sbi(PORT_CMD, RW);
}

/* ������� ������� ������� � ��������
������� � ��������� �������*/
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

/* ������� ����������� ������� � �������� �������
col - ����� ���������� �� �������������� ��� (�� 0 �� 15)
row - ����� ������ (0 ��� 1) */
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

 // ������� ������ ������� � ���������� ������� 
 void define_char(unsigned char Simvol[],unsigned char addres) 
 {
char i;
// ������� 0b01000000 + addres*8 ���������� �������� �� ������ � ������� CGRAM
lcd_putbyte(0x40+addres*8, LCD_COMMAND);
 
for(i=0;i<8;i++)
lcd_putbyte(Simvol[i],LCD_DATA);
}

#endif;