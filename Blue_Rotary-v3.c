#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/pgmspace.h>
#include "uart.h"
#include "BlueRotary.h"


//================================================================
//Define Global Variables
//================================================================
volatile char message_complete, ring_tone_flag;	//general purpse flags
volatile int message_index=0;
int dialed_number, counter;
int get_number_timeout=0;
char number_length, temp;
char phone_number[MAX_NUMBER_LENGTH];
char rotary_high;
int connected=0;
unsigned int location_350=0, location_440=0;

ISR(TIMER0_OVF_vect)
{
	cli();
    sleep_disable();
    sbi(PORTD,DT1);
    cbi(PORTD,DT2);
	sei();
    sleep_enable();
}

ISR(TIMER2_COMPA_vect) // 350 Hz buzz
{
    cli();
    sleep_disable();
    cbi(PORTD,DT1);
    OCR2A = pgm_read_byte(&(sine_table[location_350 >> STEP_SHIFT]));
    location_350 += STEP_350;
    if(location_350 >= (SINE_SAMPLES << STEP_SHIFT)) location_350 -= (SINE_SAMPLES << STEP_SHIFT);
    sei();    
    sleep_enable();
}

ISR(TIMER2_COMPB_vect) // 440 Hz buzz
{
    cli();
    sleep_disable();
    sbi(PORTD,DT2);
    OCR2B = pgm_read_byte(&(sine_table[location_440 >> STEP_SHIFT]));
    location_440 += STEP_440;
    if(location_440 >= (SINE_SAMPLES << STEP_SHIFT)) location_440 -= (SINE_SAMPLES << STEP_SHIFT);
    sei();
    sleep_enable();
}

ISR(PCINT1_vect) //HOOK
{
    cli();
    sleep_disable();
    _delay_ms(50);
    if(PINC & (1<<HOOK)) { // picked up!
        LED_OFF();
        TCCR2B |= (1<<CS20); // turn on dial tone
    }
    else { // hung up!
        TCCR2B &= ~(1<<CS20); // turn off dial tone
        counter = 0; // clear out the dialed number
        LED_ON();
    }
    sei();
    sleep_enable();
}

ISR(PCINT2_vect) // ROTARY (or EROTARY)
{
    cli();
    if(PIND & (1<<ROTARY)){
        rotary_high = 1;
        _delay_ms(20);
    }
    if ((PIND & (1<<ROTARY)) != (1 << ROTARY)){
        _delay_ms(20);
        TCCR0B &= ~(1<<CS00); // turn off dial tone
        if(rotary_high == 1){
            rotary_high = 0;
            dialed_number++;
        }
    }
    if (PIND & (1<<EROTARY)){
        dialed_number--;
        if(counter < MAX_NUMBER_LENGTH){
        phone_number[counter++] = dialed_number;
        }
        else {
            // error_ring();
        }
        for(char i = 0; i < dialed_number; i++){
            LED_ON();
            _delay_ms(200);
            LED_OFF();
            _delay_ms(200);
        }
        dialed_number = 0;
    }
    PCMSK2 &= ~((1<<PCINT19)|(1<<PCINT20));
    sei();
    PCMSK2 |= (1<<PCINT19)|(1<<PCINT20);
}

//Function: ioinit
//Purpose:	Initialize AVR I/O, UART and Interrupts
//Inputs:	None
//Outputs:	None
void ioinit(void)
{
    //1 = output, 0 = input
	DDRB = (1<<CS) | (1<<SCK) | (1<<MOSI);										//Define Port B GPIO Outputs
    PORTB = (1<<MISO);
	DDRB &= ~(1<<MISO);
    
	DDRC = (1<<BT_EN) | (1<<RING_PWR) | (1<<RING1) | (1<<RING2) |  (1<<PSTAT); 	//Define Port C Outputs
	PORTC = (1<<HOOK);
	DDRC &= ~(1<<HOOK);															//Define Port C Inputs
    
	DDRD = (1<<DT1) | (1<<DT2) | (1<< BT_RES);										//Define Port D Outputs
	PORTD = (1<<ROTARY) | (1<<EROTARY);	
    DDRD &= ~((1<<ROTARY)|(1<<EROTARY));								//Define Port D inputs
    
    //SPI Bus setup
	//SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA)|(1<<CPOL)|(1<<SPR0);	//Might need to change the phase

    uart_init(BAUD_RATE);
    
    cli();
    
    
	//Init timer for dial tone
    ASSR = 0;
    TCCR2A=(1<<WGM21)|(1<<WGM20);
    //    TCCR0B=(1<<CS00); // this turns on the timer now!
    TIMSK2 = (1<<OCIE2A)|(1<<TOIE2)|(1<<OCIE2B);
	OCR2A=(0x00);		//Load Compare Register with Delay
    OCR2B=(0x00);
    PCICR |= (1<<PCIE0)|(1<<PCIE1)|(1<<PCIE2);
    PCMSK1 |= (1<<PCINT8);
    PCMSK2 |= (1<<PCINT19)|(1<<PCINT20);
}

void interpret_message(char *message)
{
	char words[8][32];
	char *p = strtok(message, " ");
	unsigned short i=0;
	while (*p)
	{
		strcpy(words[i++], p);
		p = strtok(NULL, " ");
	}
	if(strcmp(words[0], "HFP") == 0)
	{
		
	}
	if(strcmp(words[0], "RING") == 0)
	{
		
	}
	if(strcmp(words[0], "SCO") == 0)
	{
		
	}
}

int main(void){
    char message[MAX_MESSAGE_LENGTH];	//Buffer for UART messages
    int uart_recv;
    char uart_err;
    char uart_char;
    ioinit();
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();
    message_index = 0;
    while(1){
        uart_recv = uart_getc();
        uart_err = (uart_recv & 0xff00) >> 8;
        uart_char = uart_recv & 0x00ff;
        if(!(uart_err) && message_index < MAX_MESSAGE_LENGTH && uart_char != '\r')
        {
            message[message_index++] = uart_char;
        }
        if(uart_char == '\n')
        {
            message[message_index-1]='\0';
            message_index = 0;
            interpret_message(message);
        } 
        sleep_cpu(); // we wake up when we receive a character, right?
    }
    return 0;
}
