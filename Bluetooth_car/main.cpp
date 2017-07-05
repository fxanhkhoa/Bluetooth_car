/*
 * Bluetooth_car.cpp
 *
 * Created: 7/3/2017 11:44:46 PM
 * Author : Anh Khoa
 */ 

#include <avr/io.h>
#define F_CPU 16000000
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16 * BAUD)) - 1)

#include <avr/interrupt.h>
#include <util/delay.h>

#define ratio 0.9

#ifndef sbi
#define sbi(port,bit)	port |= (1 << bit)
#endif

#ifndef cbi
#define cbi(port,bit)	port &= ~(1 << bit)
#endif

#define LATCH	4
#define DATA	5
#define SCK		7
#define BTN0	0b11111101
#define BTN1	0b11111011
#define BTN2	0b11110111

/* PORTC */
#define DIR00		4
#define DIR01		5
#define DIR0_DDR	DDRC
#define DIR0_PORT	PORTC

/* PORTD */
#define DIR10		3
#define DIR11		6
#define DIR1_DDR	DDRD
#define DIR1_PORT	PORTD

#define SERVO_CENTER		3000 +(22*12)	//Sai s? c?a c?n sensor trên xe
#define STEP				22				//B??c quay c?a servo
#define ANGLE_MAX			45				//Goc quay toi da servo
#define vach_xam			19/20			//B?ng 1 n?u ???ng line không có v?ch xám


void speed(float left, float right, float percent);
void INIT();
void handle(int goc);

int temp;

void uart_send(unsigned char c)
{
	while (!(UCSRA & (1<<UDRE))) {}; //cho den khi bit UDRE=1
	UDR = c;
}

unsigned char USART_Receive()
{
	//PORTA ^= 0xff;
	while ((UCSRA & (1 << RXC)) == 0) {}; // Do nothing until data have been received and is ready to be read from UDR
	return UDR; // Fetch the received byte value into the variable "ByteReceived"
}

int main(void)
{
	INIT();
	//DDRD = 0x00;
	//PORTD = 0x00;
	
	//DDRA = 0xff;
	//PORTA = 0x00;
	/********  USART Init *********/
	UBRRH = (103 >> 8);
	UBRRL = 103;
	
	//UCSRA = 0x00;
	//UCSRB = (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	
	/******* PWM Init **********/
	
	sei();
    /* Replace with your application code */
    while (1) 
    {
		if (temp == 1) speed(50,50,50);
		else if (temp == 2) speed(0,0,0);
		//unsigned char data = USART_Receive();
		//if (data == '1') PORTA = (1 << 0);
		for (unsigned char i = 32; i < 128; i++)
		{
			//uart_send(i);
			//_delay_ms(100);
		}
    }
}

ISR(USART_RXC_vect)
{
	uint8_t data = UDR;
	UDR = data;
	switch (data)
	{
		case '1':
			temp = 1;
			speed(50,50,100);
		break;
		case '2':
			temp = 2;
			speed(0,0,0);
		break;
		default:
			//PORTA ^= (1 << 2);
		break;
	};
}

void speed(float left, float right, float percent)
{
	left  = left  *  ratio * (percent/100.0);
	right = right *  ratio * (percent/100.0);

	if(left >= 0)
	{
		sbi(DIR0_PORT, DIR00);
		cbi(DIR0_PORT, DIR01);
		OCR1B = left*200;
	}
	else
	{
		cbi(DIR0_PORT, DIR00);
		sbi(DIR0_PORT, DIR01);
		OCR1B = -left*200;
	}
	
	if(right >= 0)
	{
		sbi(DIR1_PORT, DIR10);
		cbi(DIR1_PORT, DIR11);
		OCR2 = right*255/100;
	}
	else
	{
		cbi(DIR1_PORT, DIR10);
		sbi(DIR1_PORT, DIR11);
		OCR2 = -right*255/100;
	}
}

//=======================INITIAL=========================
void INIT()
{
	//ADC
	ADMUX=(1<<REFS0);										// 0b0100000000 Ch?n ?i?n áp tham chi?u t? chân AVCC, thêm t? ? AREF
	ADCSRA=(1<<ADEN) | (1<<ADPS2)|(1<<ADPS2)|(1<<ADPS0);	// 0b10000111 Enable ADC and set Prescaler = 128
	//read_adc_eeprom();										// T? ??ng ??c Eeprom ra khi b?t ngu?n chip
	
	//PORT
	DDRB  = 0b11110001;
	PORTB = 0b11111111;
	
	DDRC  = 0b00000000;			// Dipswitch
	PORTC = 0b11111111;
	
	DDRD  = 0b11111011;
	
	DIR0_DDR |= (1 << DIR00) | (1 << DIR01);
	DIR0_PORT = (DIR0_PORT & ~((1 << DIR00) | (1 << DIR01))) | (1 << DIR00);
	
	DIR1_DDR |= (1 << DIR10) | (1 << DIR11);
	DIR1_PORT = (DIR1_PORT & ~((1 << DIR10) | (1 << DIR11))) | (1 << DIR10);				// DIR00 = 1, DIR01 = 0, DIR10 = 1, DIR11 = 0
	
	//SPI
	SPCR = (1<<SPE)|(1<<MSTR);							//Enable spi, Master
	SPSR = (1<<SPI2X);									//SCK Mode 2X: Fosc/2
	
	//TIMER
	TCCR0=(1<<WGM01) | (1<<CS02);							// Mode 2 CTC,  Prescaler = 256
	OCR0=62;												// 1ms
	TIMSK=(1<<OCIE0);
	
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);			// SET OCR1A & OCR1B at BOTTOM, CLEAR at Compare Match (Non-invert), Mode 14 Fast PWM
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);				// Prescaler = 8
	ICR1 = 20000;											// Time Period = 10ms
	
	TCCR2=(1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);  //SET OC2 at BOTTOM, CLEAR OC2 on compare match,(non-invert), Mode 3 Fast PWM,  Prescaler = 1024
	OCR2=0;
	sei();
	
	//ENCODER
	MCUCR |= (1<<ISC11)|(1<<ISC01);
	GICR |= (1<<INT0);
}

void handle(int goc)
{
	if (goc > ANGLE_MAX)		 goc = ANGLE_MAX;
	else if(goc < -ANGLE_MAX)	 goc = -ANGLE_MAX;
	OCR1A = SERVO_CENTER+goc*STEP;
}

