#ifndef _uart328_H_
#define _uart328_H_

//#define F_CPU 16000000L

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define BUF_SIZE 128   //Исходящий буфер
#define BUF_MASK (BUF_SIZE-1)
#define IN_BUF_SIZE 128 //Входящий буфер
#define IN_BUF_MASK (IN_BUF_SIZE-1)

char buffer[BUF_SIZE]="";
char inbuf[IN_BUF_SIZE]="$"; //inner buffer of USART
volatile uint8_t ind_in=0, ind_out=0, rxind_out=0, rxind_in=0, mess = 0;
volatile uint8_t com_detect=0;  //сюда будет записана обнаруженная команда

volatile uint8_t flagof=0;

//#define TIMEOUT 100     //на случай если команда так и не принята



//#define sbit(sfr, bit) (sfr|=(1<<bit))
//#define cbit(sfr, bit) (sfr&=~(1<<bit))
 
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU/(USART_BAUDRATE*16UL)))-1)
// compatibility with megaXX8 processors

	#define UDR					UDR0
	#define UCR					UCSR0B
	 #define UCSRA					UCSR0A
	 #define UCSRB					UCSR0B
	 #define UCSRC					UCSR0C
	 #define UCSRE					UCSR0E
	 #define UCSEL					UCSE0L
     #define URSEL					UMSEL00
	 #define UCSZ1					UCSZ01
	 #define UCSZ0					UCSZ00
	 #define UDRE					UDRE0
	#define RXCIE				RXCIE0
	#define TXCIE				TXCIE0
	#define UDRIE				UDRIE0
	#define RXC					RXC0
	#define TXC					TXC0
	#define RXEN				RXEN0
	#define TXEN				TXEN0
	#define UBRRL				UBRR0L
	#define UBRRH				UBRR0H
	//#define SIG_UART_TRANS		SIG_USART_TRANS
	//#define SIG_UART_RECV		SIG_USART_RECV
	//#define SIG_UART_DATA		SIG_USART_DATA  
 


void uart_init(){ 
 UCSRB |= (1<<RXEN)  | (1<<TXEN);
 UCSRC |= (1<<UCSZ0) | (1<<UCSZ1);
 UBRRH  = (BAUD_PRESCALE >> 8);
 UBRRL  = BAUD_PRESCALE;
 //enable RX/TX interrupts
 UCSRB |= ((1<<RXEN) | (1<<TXEN) | (1<<RXCIE));
//sei();
}
 // function to send data - NOT REQUIRED FOR THIS PROGRAM IMPLEMENTATION
void uart_transmit (unsigned char data)
{
    while (!( UCSRA & (1<<UDRE)));            // wait while register is free
    UDR = data;                             // load data in the register
}
  
// function to receive data
unsigned char uart_recieve (void)
{
    while(!(UCSRA) & (1<<RXC));           // wait while data is being received
    return UDR;                             // return 8-bit data
}


void uart_putstring(char* StringPtr){
 
while(*StringPtr != 0x00){
 uart_transmit(*StringPtr);
 StringPtr++;}
 
}

/*
unsigned unsigned char buff[40];

unsigned unsigned char* uart_getstring(){
unsigned unsigned char buff1[5]={'A','1','s','3','5'};
//volatile unsigned unsigned char ch;
unsigned unsigned char i=0;


do {
		  buff[i]=uart_recieve();
		  i++;
    }while(i<40);
;
//itoa (i,buf,1);
//uart_transmit(buf);
//uart_putstring(buff);

	return *buff1;
}

*/



ISR (USART_RX_vect){
   
	 //disable RX/TX interrupts  */
	 //cli();
    UCSRB &= ~((1<<RXCIE) | (1<<UDRIE)); 
   unsigned char tmp;
   tmp=uart_recieve();

//Теперь чтобы проверить свободен ли  UDR - регистр данных
//USART - достаточно сделать логическое "И" числа в регистре  UCSRA  с маской  (1<<UDRE)
if (!(UCSRA & (1<<UDRE))==0)        
		if ((tmp ==(0x0D)))        //получен конец команды - <enter>

                {
                mess++; //one more message
                inbuf[rxind_in++] = '$'; //вставляем разделитель в буфер
                rxind_in &= IN_BUF_MASK;

			flagof=1;
                }
    else 
                {
                if (tmp != 0x0A) //очистка непонятного символа с модуля
                        {
                        inbuf[rxind_in++] = tmp;   //записываем в буфер
                        rxind_in &= IN_BUF_MASK;

                        };
                }
asm("nop");	
asm("nop");	
asm("nop");	
//enable RX/TX interrupts
 UCSRB |= ((1<<RXEN) | (1<<TXEN) | (1<<RXCIE));
//sei();


}



/*
//sending RS232 with interupt
void SendByte(unsigned char byte)
        {
        buffer[ind_in++] = byte;
        ind_in &= BUF_MASK;
        }

void SendStr(unsigned char *string)
        {
        while (*string!='\0')  //check if End
                {
                SendByte(*string);
                string++;
                }
        }





//Sending data from buffer
ISR (USART_UDRE_vect)           
        {
        UDR = buffer[ind_out++];  //запись из буфера
        ind_out &= BUF_MASK;      //проверка маски кольцевого буфера
        if (ind_in == ind_out)  //если буфер уже пуст
                {
                UCSRB &= ~(1<<UDRIE); //disable instrupt UDR empty
                UCSRB |= (1<<RXEN);   //разрешение приёма после окончания отправки
                }
        sei ();
        }
*/

void inbuf_clr(){
for(int i=0 ; i<rxind_in;i++) inbuf[i] = 0x00;
rxind_in=0;
}


#endif;
