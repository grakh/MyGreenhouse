#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/wdt.h> // здесь организована работа с ватчдогом
//функция get_mcusr(), которая должна вызываться сразу после сброса.
//после перезагрузки, которая была вызвана watchdog, контроллеры последних выпусков оставляют включенным watchdog на минимальный период, т.е. 15ms. 
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void){
mcusr_mirror = MCUSR;
MCUSR = 0;
wdt_disable();
}

#include <util/delay.h>
#include <avr/interrupt.h >
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <LCD_2.h>
#include "DHT11-master.h"
#include "uart328.h"
#include <avr/eeprom.h>
#include <avr/sleep.h> // здесь описаны режимы сна


	#define ADC_VREF_TYPE 0x60
	#define NULLIC 0

	#define CYCLE 5000
	#define SERVO_PORT PORTB
	#define SERVO_PIN1 PB3
	#define SERVO_PIN2 PB4
	#define RELE PB5
	#define PUMP PD3
	#define EMERGENCY PD2
	#define WIND 2
	#define WATER PC5 
	#define PHOTO 3
	#define SM 4


   
    volatile int angle1 = 800;
    volatile int angle2 = 800;
    volatile int angle3 = 800;
    volatile int angle4 = 800;
    volatile int takt = 0;
	//volatile unsigned long millis=1;

	unsigned char takti = 0;
	unsigned char sek = 0;
	unsigned char min=0, smin=0;
	unsigned char hour=0, shour=0;
	char water, servo=-1;
	char clock[6];
	char temp[17];
	char flagVent=0, flagRele=0, flagWater=0, flagCircle=0, flagAuto=0, flagLCD=0, flagWork=0, flagGSM=1;
	unsigned char flag_but=0, autoFlag=0, interKey=0;

	//char gsm[IN_BUF_SIZE];

	uint16_t tmpSleep=5000, tmpRele=2000, tmpWater=5000, timeSetting=440;
	uint16_t maxWater EEMEM, maxRele EEMEM, maxSleep EEMEM, mAngle EEMEM;
	uint8_t humW EEMEM, smsTime EEMEM;
	uint8_t numVent=0;

	struct Item {
	int stat;
	uint16_t wind;
	int tem;
	int hum;
	int lux;
	int sm;
	} *data;


int read_adc(int);
void timer1_start();
void timer1_stop();
//int Wait(long);
void GSM_init();
void gsm_sms(void);
void gsm_ring(void);
void gsm_bal(void);
void autom(void);
void working(void);
void TimeGSM(void);
uint8_t list(char[], uint8_t);

void clockUpdate(){
sprintf(clock, "%02d:%02d", hour, min);
lcd_gotoxy(11,0);
lcd_putstring(clock);
}

char automatic(unsigned char key){
 flagAuto=1;

  return 1;
};

void clrLCD1(){
lcd_gotoxy(0,1);
lcd_putstring("                ");
};

void autom(){
  //lcd_gotoxy(0,1);
  //lcd_putstring("                ");
//char temp[15];

sleep_disable();

uint8_t sensorVal=0;
uint8_t oldVal=0;
uint16_t sum=0;

  for (uint16_t i3=0; i3<10000; i3++){
    sensorVal = (PINC>>WIND)&1;

    if (sensorVal!=oldVal) {sum++;}
    oldVal=sensorVal;
    _delay_ms(1);
      }
data->wind = sum;
data->tem = dht11_gettemperature();
data->hum = 100-dht11_gethumidity();
data->lux = read_adc(PHOTO);
data->sm = 100-read_adc(SM)*100/255;


  //sum = sum/10*1.41;

//uart_transmit(data->wind);
//sprintf(temp, "T=%02d\1 V=%02d%% P%02d%%", data->tem, data->hum, data->sm);
//uart_putstring(temp);
//uart_transmit(0x0D);
//wdt_reset(); // сбрасываем
//wdt_enable(WDTO_8S);
flagWork=1;
flagAuto=0;
};

void working(){

if (data->tem >=30 || data->hum > 60) {
	if (data->wind < 90 ) 						flagVent=1;
	if (data->wind >= 90 && data->wind < 180) 	flagVent=2;
	if (data->wind >=180) 						flagVent=3;
	}
if (data->tem <= 25) 							flagVent=3;

if (data->sm <= eeprom_read_byte(&humW) && data->lux <= 170) 		flagWater=1;
flagWork=0;
sprintf(temp, "T=%02d\1 V=%02d%% P%02d%%", data->tem, data->hum, data->sm);
//uart_putstring(temp);
//uart_transmit(0x0D);
};

void GSM_init(){
_delay_ms(12000);
		uart_putstring("AT+CMGF=1"); uart_transmit(0x0D);
		_delay_ms(100);
		uart_putstring("AT+CSCS=\"GSM\""); uart_transmit(0x0D);
		_delay_ms(100);
		uart_putstring("AT+CLIP=1"); uart_transmit(0x0D);
		_delay_ms(100);
		uart_putstring("AT+CNMI=2,2"); uart_transmit(0x0D); //??????????? ???
		_delay_ms(100);
		//uart_putstring("AT+CUSD=1,\"*105*00#\",15"); uart_transmit(0x0D); //??????????? ???
}

char Ventilation(unsigned char key){

	flagVent=1;
  return 1;
};

char VentOFF(unsigned char key){

	flagVent=3;
  return 1;
};


void Vent100(){
clrLCD1();
lcd_gotoxy(0,1);
lcd_putstring("Open 100%");
if (servo!=2){
numVent++;
    timer1_start();
		_delay_ms(2);
angle1 = (uint16_t)eeprom_read_word(&mAngle);
	
		_delay_ms(2000);
angle2 = 800;
		_delay_ms(2000);

	timer1_stop();
	}
	servo=2;
	flagVent=0;

};

void Vent50(){
clrLCD1();
lcd_gotoxy(0,1);
lcd_putstring("Open 50%");
if (servo!=1){
numVent++;
    timer1_start();
		_delay_ms(100);
angle1 = (uint16_t)eeprom_read_word(&mAngle)/2;
		_delay_ms(8000);
angle2 = angle1;
		_delay_ms(8000);

	timer1_stop();
	}
	servo=1;
	flagVent=0;

};

void Vent0(){
clrLCD1();
lcd_gotoxy(0,1);
lcd_putstring("Close");
if (servo!=0){
    timer1_start();
		_delay_ms(2);
angle1 = 800;
	_delay_ms(4000);
	
angle2 = (uint16_t)eeprom_read_word(&mAngle);
		_delay_ms(4000);

	timer1_stop();
	}
	servo=0;
	flagVent=0;

};

char Watering(unsigned char key){
flagWater=1;
  return 1;
};

char Rele(unsigned char key){

flagRele=1;

  return 1;
};

void flRele(){
if (flagRele==1){
//lcd_gotoxy(0,1);
//lcd_putstring("Rele ON");
tmpRele=(uint16_t)eeprom_read_word(&maxRele);
PORTB |= (1<<RELE);
flagRele=2;
};
tmpRele--;
if (tmpRele==0){
PORTB &=~(1<<RELE);
clrLCD1();
lcd_gotoxy(0,1);
lcd_putstring("Rele OFF");
flagRele=0;
 };
}

void flWater(){

water = (PINC>>5)&1;

if ((flagWater==1) && (water==1)){
//lcd_gotoxy(0,1);
//lcd_putstring("Water ON");
tmpWater = (uint16_t)eeprom_read_word(&maxWater);

PORTD |= (1<<PUMP);
flagWater=2;
};
tmpWater--;
if (tmpWater==0 || water==0){
PORTD &=~(1<<PUMP);
clrLCD1();
lcd_gotoxy(0,1);
lcd_putstring("Water OFF");
shour=hour;
smin=min;
flagWater=0;
 };
}

char emergency(unsigned char key){
PORTD |= (1<<EMERGENCY);
_delay_ms(100);
PORTD &=~(1<<EMERGENCY);
_delay_ms(100);
GSM_init();
return 1;
}


char timeSet(unsigned char key){
hour = list("Hour",24);
min = list("Min",60);
clockUpdate();
return 1;
}

char tWater(unsigned char key){
eeprom_write_word(&maxWater, list("Water, m/2",120)*timeSetting);

return 1;
}

char tRele(unsigned char key){

eeprom_write_word(&maxRele, list("Rele, m/2",240)*timeSetting);

return 1;
}

char sAngle(unsigned char key){

eeprom_write_word(&mAngle, 800+list("Servo angle",180)*13);

return 1;
}

char tLight(unsigned char key){

eeprom_write_word(&maxSleep, list("Time Sleep",120)*timeSetting);

return 1;
}

char hWater(unsigned char key){

eeprom_write_byte(&humW, list("Humidate",100));

return 1;
}

char tsms(unsigned char key){

eeprom_write_byte(&smsTime, list("Sms Time",24));

return 1;
}

struct MenuItem{
  unsigned char parent,child; //номер "родителя" и "потомка" в массиве
  unsigned char prev,next; //номер "соседей" в массиве
  char (*func)(unsigned char); //указатель на функцию, вызываемую при выборе элемента char (*func)(unsigned char);
  char caption[15]; //название пункта (в строку влазит 16 символов, но справа и слева оставим место для рамок, а последний символ по стандарту Си должен быть нулем)
};

struct MenuItem menu[]={
  {0, 3, 2, 1, 		NULLIC, 	"Manual"},
  {1, 0, 0, 2, 		automatic,  "Automat"},
  {2, 8, 1, 0, 		NULLIC,  	"Settings"},

  {0, 3, 7, 4, 		Ventilation,"Ventilation ON"},
  {0, 4, 3, 5, 		VentOFF,	"Ventilation OFF"},
  {0, 5, 4, 6, 		Watering,  	"Watering"},
  {0, 6, 5, 7, 		Rele,  		"Rele"},
  {0, 7, 6, 3, 		emergency,  "GSM Reset"},

  {2, 8, 14, 9, 	timeSet, 	"Time Setting"},
  {2, 9, 8, 10,  	tWater,  	"Watering time"},
  {2, 10, 9, 11,  	tRele,  	"Rele time"},
  {2, 11, 10, 12,  	sAngle,  	"Angle servo"},
  {2, 12, 11, 13, 	tLight,  	"Lightening time"},
  {2, 13, 12, 14, 	hWater, 	"H for watering"},
  {2, 14, 13, 8, 	tsms, 		"Time to SMS"}
};


unsigned char posi=0;
void menu_redraw(unsigned char pos){
if (pos!=posi){
  lcd_gotoxy(0,0);
  lcd_putstring("           ");
  lcd_gotoxy(0,0);
  lcd_putstring("["); lcd_putstring(menu[ menu[pos].parent ].caption); lcd_putstring("]"); //1 строка - название родительского пункта в квадратных скобках, чтобы не путаться
  //lcd_gotoxy(LCD_STR_2);
  //lcd_putstring(" "); lcd_putstring(menu[ menu[pos].prev ].caption); lcd_putstring(" "); //2 строка - название предыдущего пункта
clrLCD1();
  lcd_gotoxy(0,1);
  lcd_putstring(">"); lcd_putstring(menu[ pos ].caption); //3 строка - название выделенного пункта, в угловых скобках
  //lcd_gotoxy(LCD_STR_4);
  //lcd_putstring(" "); lcd_putstring(menu[ menu[pos].next ].caption); lcd_putstring(" "); //4 строка - название следующего пункта

  };
    posi=pos;
};

//обработка логики. На вход принимает нажатую клавишу.
void Menu(unsigned char key){
  
  static unsigned char menu_pos=0;
  static char (*func)(unsigned char) = NULLIC;
  if( func == NULLIC){
           if( key == 2)menu_pos = menu[menu_pos].parent;
      else if( key == 3)menu_pos = menu[menu_pos].child;
      else if( key == 4)menu_pos = menu[menu_pos].prev;
      else if( key == 1)menu_pos = menu[menu_pos].next;
      else if( key == 5)func = menu[menu_pos].func;
    menu_redraw(menu_pos);
  }else{
    if( func(key) )func = NULLIC;
  }

};

unsigned char degree[8] = // ???????? ?????? ???????
	{
   0B00111,
   0B00101,
   0B00111,
   0B00000,
   0B00000,
   0B00000,
   0B00000,
	};

unsigned char degreem[8] = // ???????? m/c
{
   0B10110,
   0B11110,
   0B10010,
   0B00000,
   0B00111,
   0B00100,
   0B00111,
};

/*
int Wait(long w) {
	for(long i=0;i<w;i++){
	//asm("nop");
_delay_loop_2(0);
_delay_loop_2(0);
_delay_loop_2(0);
 		}
 return 1;
	}

uint8_t flag1=0;
uint8_t wait(unsigned int times){
	if (flag1==0) {millis=0; flag1=1;}
	if (millis<times) return 0;
	flag1=0;
	return 1;
}

int getADC(int pin) //?????????? ???
    {
       //ADMUX=pin | (ADC_VREF_TYPE & 0xff);
	   ADMUX |=(1<<ADLAR);
     ADMUX &= 0x0F;
	 ADMUX |= pin;	 // ??????? ?????
	     // Delay needed for the stabilization of the ADC input voltage
        _delay_us(10);
    ADCSRA|=(1<<ADSC); //?????? ??????????????
    while (!(ADCSRA&_BV(ADIF))); //????????? ????????? ??????????????
    return (ADCL|ADCH<<8);
	}
*/
int read_adc(int adc_input) {
	  // ADMUX |=(1<<ADLAR);
        ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
        // Delay needed for the stabilization of the ADC input voltage
        _delay_us(10);
        // Start the AD conversion
        ADCSRA|=0x40;
        // Wait for the AD conversion to complete
        while ((ADCSRA & 0x10)==0);
        ADCSRA|=0x10;
        return ADCH;
	} 

unsigned char read_key() {
	unsigned char key_adc=0, last_key=0;

	last_key=read_adc(0);
	_delay_ms(60);
	key_adc=read_adc(0);

if (last_key==key_adc){
last_key=0;

	if                     (key_adc <=  29)  return 1;  // = 0 
	if ((key_adc >= 30) && (key_adc <= 69)) return 2; // = 1/2 ?? 255
	if ((key_adc >= 70) && (key_adc <= 119)) return 3;   // = 2/3 ?? 255
	if ((key_adc >= 120) && (key_adc <= 169)) return 4;   // = 3/4 ?? 255
	if ((key_adc >= 170) && (key_adc <= 239)) {if(flagLCD==1) {lcd_light(1);flagLCD=0; sleep_disable();
													tmpSleep = (uint16_t)eeprom_read_word(&maxSleep);
													lcd_gotoxy(0,1); lcd_putstring(temp);
													return 0;} return 5;}     // = 4/5 ?? 255
	if (key_adc>240) return 0;
	}
	return 0;
} // ---------------------------------------------------------------------------------------



ISR(TIMER0_COMPA_vect){

//if (millis <= 1339201) millis++; else millis=1;
	takti++;
	//if (takti>=62)  flag_but=1;
	if (takti>=124) {sek++; takti=0x00; //8 000 000 \ 256 \ 250 = .
						wdt_reset();
						}
	if (sek>=60)   {min++; sek=0x00;}
	if (min>=60)   {hour++; min=0x00;}
	if (hour>=24)  hour=0x00;

}
     
void timer1_init() {
    OCR1A = 20000;
    TCCR1A = 0;
    TCCR1B |= (1 << WGM12);
	}
     
void timer1_start() {
    TCNT1 = 0;
    TCCR1B |= (1<<CS11);
    TIMSK1 |= (1 << OCIE1A);
	}
     
void timer1_stop() {
    TCCR1B &= 0b11111000;
    TIMSK1 &= !(1 << OCIE1A);
	}

void acp_init(){
	//       источник опорн напр,сдвиг регист,       ножки ADC0...ADC7
	ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	//      on/off, зап. одного преоб.прерывн преобр, разреш преобр,коэф.дел = FCPU/128
	ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

}

void timer0_init() {
	TIMSK0 |= (1<<OCIE0A); //compare match
    TCCR0A = 0;           // set timer2 normal mode   
    TCCR0A |=(1<<WGM01)|(0<<WGM00); // CTC 
    TCCR0B |= (1<< CS02)|(0<< CS01)|(0<< CS00); //256
	//TIFR0 |= (1<<TOV0); //??? ???????????? ???????? ????????
	TIFR0 |= (1<<OCF0A); // 
    OCR0A = 251;
	TCNT0 = 0;

	//TIMSK0 |= _BV(TOIE0); //on переполнение
	}
     
int main () {

 // char key,oldkey;

cli();
lcd_init();
lcd_light(1);
uart_init();
lcd_clear();

define_char(degree,1);
define_char(degreem,2);

	DDRB |= (1<<SERVO_PIN1); // pinMode(13,OUTPUT); ? Wiring
	DDRB |= (1<<RELE); // pinMode(13,OUTPUT); ? Wiring
	DDRB |= (1<<SERVO_PIN2); // pinMode(13,OUTPUT); ? Wiring
	DDRD |= (1<<PUMP); // pinMode(13,OUTPUT); ? Wiring
	DDRD |= (1<<EMERGENCY); // pinMode(13,OUTPUT); ? Wiring
	DDRC &=~(1<<WATER);
	DDRC &=~(1<<PC2); //wind
	PORTC &=~ (1<<PC2);


	SERVO_PORT &= ~(1<<SERVO_PIN1);
	SERVO_PORT &= ~(1<<SERVO_PIN2);
	//PORTC |=(1<<WATER);

timer1_init();
timer0_init();
acp_init();
        wdt_reset(); // сбрасываем
        wdt_enable(WDTO_2S); // разрешаем ватчдог 8 сек
//        WDTCSR |= _BV(WDIE); // разрешаем прерывания по ватчдогу. Иначе будет резет.
        set_sleep_mode(SLEEP_MODE_IDLE); // если спать - то на полную

    sei();
GSM_init();
//char delim[]="\",$@";
//char delim1[]=",";
//char *istr, *tmm, th[2], tm[2];
    //timer1_start();
	timer1_stop();

	//eeprom_write_word(&maxWater, tmpWater);
	//eeprom_write_word(&maxRele, tmpRele);
	//eeprom_write_word(&maxSleep, tmpSleep);
	//eeprom_write_word(&mAngle, 2100);
	//eeprom_write_byte(&humW, 40);
	//eeprom_write_byte(&smsTime, 19);

	//-----------------------------------------------------------------------------------

while (1){

Menu(read_key());

clockUpdate();


tmpSleep--;
if(tmpSleep==0) {
				lcd_light(0);
				flagLCD=1;
				sleep_enable(); // разрешаем сон
				sleep_cpu(); // спать!
				}
//if(millis == millis+(milSleep--)) {lcd_light(0); flagLCD=1; milSleep=0;}

if ((rxind_in>0) && (flagCircle==1)){

if (strstr (inbuf,"+CMT")!=0) {gsm_sms();}

if (strstr (inbuf,"+CLIP")!=0) {gsm_ring();}

if ((strstr (inbuf,"+CUSD:")!=0)&&(strstr (inbuf,"r.")!=0)) {gsm_bal();}

if (strstr (inbuf,"+CREG:")!=0) if (strstr (inbuf,",1")==0) emergency(5);

inbuf_clr();
flagCircle=0;
		}
if ((rxind_in>0) && (flagCircle==0)) flagCircle=1;

if (flagAuto==1) autom();
if (flagRele!=0) flRele(); 
if (flagWater!=0) flWater(); 
if (flagVent==1) Vent100();
//if (flagVent==2) Vent50();
if (flagVent==3) Vent0();
if (flagWork) working();

if (min==6 && sek==11) flagAuto=1;
if (hour == eeprom_read_byte(&smsTime) && min == 10 && sek == 15) TimeGSM();
if (hour == 21 && min == 20 && sek == 15 && takti<10) {flagGSM=0; uart_putstring("AT+CUSD=1,\"*100#\",15"); uart_transmit(0x0D);}
if (min == 16 && sek == 15 && takti<10) {uart_putstring("AT+CREG?"); uart_transmit(0x0D);}

if (hour == 23 && min == 59 && sek == 15) {shour=0; smin=0; numVent=0;}

    }

   return 0;
}







void gsm_sms(void){

//uart_putstring(inbuf);
//uart_transmit(0x0D);
int ai=0;
//	char gsmr[strlen(inbuf)];
	char *istr, *tmm, *thm, *gsm, th[3], tm[3];

 //strcpy(gsmr,inbuf);

if (strstr (inbuf,"9778056480")!=0 || strstr (inbuf,"9295789784")!=0 || strstr (inbuf,"9295657305")!=0) {

istr=strchr(inbuf, '/');
thm=strchr(istr, ',');
strncpy(th,thm+1, 2);
//strcat(th, 0x00);
ai=atoi(th);
if (ai!=0){
hour=ai;
tmm=strchr(istr, ':');
strncpy(tm,tmm+1, 2);
//strcat(tm,0x00);
min=atoi(tm);
	}
gsm=strchr(istr, '$');

if (strstr(gsm,"$R")!=0) flagRele=1;
if (strstr(gsm,"$W")!=0) flagWater=1;
if (strstr(gsm,"$V")!=0) flagVent=1;
if (strstr(gsm,"$C")!=0) flagVent=3;
if (strstr(gsm,"$A")!=0) TimeGSM();
if (strstr(gsm,"$E")!=0) emergency(5);

uart_putstring(gsm);
uart_transmit(0x0D);

	}

	_delay_ms(100);
uart_putstring("AT+CMGD=1,4\r");


//return *gsm;

}

void TimeGSM(){
 char temps[128];
water = (PINC>>5)&1;
	sprintf (temps, "Temperatura=%02d°C,\nVlajnost=%02d%%,\nPochva=%02d%%,\nVremya Poliva %02d:%02d,\nWater=%d,\nVent=%d", data->tem, data->hum, data->sm, shour, smin, water, numVent);
	_delay_ms(900);
	uart_putstring("AT+CMGS=\"+79295789784\"\r");
	_delay_ms(100);
	uart_putstring(temps);_delay_ms(10); uart_transmit(0x1A);
}

void gsm_ring(){

 char temps[128];

if (strstr (inbuf,"9778056480")!=0 || strstr (inbuf,"9165300092")!=0) {
	uart_putstring("AT+CHUP\r");
	flagRele=1;

	}
if (strstr (inbuf,"9295789784")!=0 || strstr (inbuf,"9295789785")!=0) {
	uart_putstring("AT+CHUP\r");
	flagRele=1;

	}

if (strstr (inbuf,"9295657305")!=0) {
 
	uart_putstring("AT+CHUP\r");
water = (PINC>>5)&1;
sprintf (temps, "Temperatura=%02d°C,\nVlajnost=%02d%%,\nPochva=%02d%%,\nVremya Poliva %02d:%02d,\nWater=%d,\nWind=%u,\nLux=%d,\nVent=%d", data->tem, data->hum, data->sm, shour, smin, water, data->wind, data->lux, numVent);
	_delay_ms(900);
	uart_putstring("AT+CMGS=\"+79295657305\"\r");
	_delay_ms(100);
	uart_putstring(temps);_delay_ms(10);uart_transmit(0x1A);

	}
/*
if (strstr (inbuf,"9295789784")!=0) {
 
	uart_putstring("AT+CHUP\r");

sprintf (temps, "Temperatura=%02d°C,\nVlajnost=%02d%%,\nPochva=%02d%%,\nVremya Poliva %02d:%02d,\nWater=[%d]", data->tem, data->hum, data->sm, shour, smin, water);
	_delay_ms(900);
	uart_putstring("AT+CMGS=\"+79295789784\"\r");
	_delay_ms(100);
	uart_putstring(temps); uart_transmit(0x1A);

	}
*/
}

void gsm_bal(){

 char *istrb, *bal;
 //unsigned char balance;
istrb=&inbuf[1];
 bal=strtok (istrb,"\",.");

 bal=strtok (NULL,"\",.");

//	bal=strsep(istrb,'.'); ("AT+CUSD=1,\"*144*9778056480#\",15")
if((atoi(bal))<20) {uart_putstring("AT+CUSD=1,\"*144*9778056480#\",15"); uart_transmit(0x0D);}//sprintf (istrb, "est, %d r.", balance);
//lcd_gotoxy(0,1); lcd_putstring(istrb);
//	uart_transmit(0x0D);
}

     
ISR(TIMER1_COMPA_vect) {
    if (takt == 0) {
    SERVO_PORT |= (1<<SERVO_PIN1);
    OCR1A = angle1;}
    if (takt == 1) {
    SERVO_PORT &= ~(1<<SERVO_PIN1);
    OCR1A = CYCLE - angle1;}
    if (takt == 2) {
    SERVO_PORT |= (1<<SERVO_PIN2);
    OCR1A = angle2;}
    if (takt == 3) {
    SERVO_PORT &= ~(1<<SERVO_PIN2);
    OCR1A = CYCLE - angle2;}

    takt = takt + 1;
    if (takt == 9) takt = 0;
	};

uint8_t list(char *li, uint8_t maxber){
uint8_t namber=0;
char nam[3];
clrLCD1();
lcd_gotoxy(0,1); lcd_putstring(li);
while(1){

switch (read_key()){

	case 2: if (namber < maxber-1) namber++; else namber=0; break;
	case 3: if (namber > 0) namber--; else namber=maxber-1; break;

	case 5: clrLCD1(); return namber; break;
		}
sprintf (nam, "%3d", namber);
lcd_gotoxy(13,1); lcd_putstring(nam);
_delay_ms(10);
	}
}

//ISR (WDT_vect) {
//uart_putstring("WDTC"); uart_transmit(0x0D);
//        PORTB ^= _BV(PB4); // переключаем светодиод
//        WDTCSR |= _BV(WDIE); // разрешаем прерывания по ватчдогу. Иначе будет резет.
//}
