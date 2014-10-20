#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>


// Command (6 bits)
// Value (10 bits)
// 0b000010--  Output off
// 0b000011--  Output on
// 0b000100--  Set Current (CC)
// 0b000101--  Set Voltage (CV)
// 0b000010--  Reserved
// 0b000111--  Set Voltage (Protect)

// Output:
// 0b111111-- -------- Voltage
// 0b111110-- -------- Current

#define UART_HI() (PINB&0b100)
#define UART_HI_BIT() ((PINB&0b100)>>2)

#define UART_1() sbi(PORTB, 2)
#define UART_0() cbi(PORTB, 2)

#define UART_PCINT_ISR isr_name

#define LED_ON()  sbi(PORTB, 3)
#define LED_OFF() cbi(PORTB, 3)
#define PULSE() { sbi(PORTB, 0); cbi(PORTB, 0);}


// Fixed 115200 baud/s on 8 MHz CPU
// 1 clk cycle = 0.125 us
// 8.68 us/symbol
// 69.4 clk cycles per symbol

void uart_write_byte_block(uint8_t byte)
{
	UART_0();
	_delay_us(8.68-0.750);
	for(uint8_t i = 8; i > 0; --i)
	{
		if(byte & 1)
			UART_1();
		else
			UART_0();
		_delay_us(8.68-0.875);
		byte >>= 1;
	}
	UART_1();
	_delay_us(8.68*8);
}

uint8_t uart_read_byte_block(uint8_t* out)
{
	uint8_t byte = 0;
	while(1)
	{
		if(!UART_HI())
		{
			_delay_us(1.0);
			if(!UART_HI())
				break;
		}
	}

	LED_ON();
	_delay_us(3.7);

	for(uint8_t i = 8; i > 0; --i)
	{
		_delay_us(7.7);
		byte >>= 1;
		if(UART_HI())
			byte |= 0b10000000;
		else
			__asm__ __volatile__("nop");
	}

	_delay_us(7.8);
	LED_OFF();

	PULSE();
	if(!UART_HI())
		return 1;
	*out = byte;
	return 0;
}

ISR(UART_PCINT_ISR)
{
	uart_read_byte()
}

#define PWM_MIN 70
#define PWM_MAX 220
#define MAX_CHANGE 10

int main()
{
	uint16_t i_setpoint;
	uint16_t v_max;
	uint16_t v_min;
	uint16_t last_v;
	uint16_t last_i;
	uint8_t pwm;

	DDRB  = 0b00001101; // Data out, debug pulse out, led out
	PORTB = 0b00010110; // Data input pull-up, uart out high, jumper pull-up

	while(1)
	{
		ADMUX = CURRENT;
		while(!ADC_READY);
		cli();
		last_i = ADC;
		sei();

		ADMUX = VOLTAGE;
		while(!ADC_READY);
		cli();
		last_v = ADC;
		sei();

		if(last_v > v_max)
		{
			if(i_setpoint) i_setpoint--;
		}
		else if(last_v < v_min)
		{
			if(i_setpoint < 1023) i_setpoint++;
		}

		int16_t i_err = (int16_t)last_i - (int16_t)i_setpoint;
		i_err >>= 1;
		if(i_err > MAX_CHANGE) i_err = MAX_CHANGE;
		else if(i_err < -MAX_CHANGE) i_err = -MAX_CHANGE;

		int16_t pwm_tmp = pwm;
		pwm_tmp += i_err;
		if(pwm_tmp > PWM_MAX)
			pwm = PWM_MAX;
		else if(pwm_tmp < PWM_MIN)
			pwm = PWM_MIN;
		else
			pwm = pwm_tmp;


	}

	return 0;
}


