#include "leds.h"

uint8_t led_pins[3];
volatile uint32_t led_pin_mask[3] = {0, 0, 0};
uint8_t led_driver;
uint32_t led_driver_mask = 0;
volatile int led_pulse_length = 0;
uint8_t led_output_driver_enabled = 1;
uint8_t led_default_r = 220;
uint8_t led_default_g = 30;
uint8_t led_default_b = 250;

void start_leds_timer()
{
	NRF_TIMER4->MODE = 0;
	NRF_TIMER4->BITMODE = 3; //32 bits
	NRF_TIMER4->PRESCALER = 5;
	NRF_TIMER4->CC[0] = 15000; //r
	NRF_TIMER4->CC[1] = 80; //g
	NRF_TIMER4->CC[2] = 10; //b
	NRF_TIMER4->CC[3] = 0xFFFFFFFF; //driver, not used with timer anymore
	NRF_TIMER4->CC[4] = 16384;
	NRF_TIMER4->SHORTS = 0b10000; //clear on compare 4

	NRF_TIMER4->INTENSET = 0b11111<<16; 
	NVIC_EnableIRQ(TIMER4_IRQn);

	NRF_TIMER4->TASKS_START = 1;
}

uint8_t pending_color_change = 0;

int pending_r, pending_g, pending_b;

void TIMER4_IRQHandler(void)
{
	if(NRF_TIMER4->EVENTS_COMPARE[4])
	{
		NRF_TIMER4->EVENTS_COMPARE[4] = 0;
		NRF_GPIO->OUTCLR = led_pin_mask[0] | led_pin_mask[1] | led_pin_mask[2];		
		if(pending_color_change)
		{
			NRF_TIMER4->CC[0] = pending_r;
			NRF_TIMER4->CC[1] = pending_g;
			NRF_TIMER4->CC[2] = pending_b;
			pending_color_change = 0;
		}
		if(led_pulse_length)
		{
			led_pulse_length--;
			if(led_pulse_length == 0)
			{
//				leds_set(0, 0, 0);
				NRF_GPIO->OUTSET = led_pin_mask[0] | led_pin_mask[1] | led_pin_mask[2];
				led_pin_mask[0] = 0;
				led_pin_mask[1] = 0;
				led_pin_mask[2] = 0;
				NRF_TIMER4->CC[0] = 0xFFFF;
				NRF_TIMER4->CC[1] = 0xFFFF;
				NRF_TIMER4->CC[2] = 0xFFFF;
			}
		}
	}
	if(NRF_TIMER4->EVENTS_COMPARE[2])
	{
		NRF_TIMER4->EVENTS_COMPARE[2] = 0;
		NRF_GPIO->OUTSET = led_pin_mask[2];
	}
	if(NRF_TIMER4->EVENTS_COMPARE[1])
	{
		NRF_TIMER4->EVENTS_COMPARE[1] = 0;
		NRF_GPIO->OUTSET = led_pin_mask[1];
	}
	if(NRF_TIMER4->EVENTS_COMPARE[0])
	{
		NRF_TIMER4->EVENTS_COMPARE[0] = 0;
		NRF_GPIO->OUTSET = led_pin_mask[0];
	}
}

void leds_init(int pin_r, int pin_g, int pin_b, int pin_driver)
{
	int lp[3];
	lp[0] = pin_r;
	lp[1] = pin_g;
	lp[2] = pin_b;
	led_driver = pin_driver;
	if(pin_driver < 0xFF)
	{
		led_driver_mask = 1<<led_driver;
		NRF_GPIO->DIRSET = led_driver_mask;
		NRF_GPIO->OUTCLR = led_driver_mask;
	}
	
	for(int x = 0; x < 3; x++)
	{
		if(lp[x] < 0) continue;
		led_pins[x] = lp[x];
		led_pin_mask[x] = 1<<led_pins[x];
		NRF_GPIO->DIRSET = led_pin_mask[x];
	}	
	start_leds_timer();
}
 
int val_to_cc(int val)
{
	int v2 = val*val;
	v2 >>= 2;
	if(v2 == 0) v2 = 1;
	if(v2 > 16384) v2 = 16384; 
	return v2;
}

void leds_set_driver(int val)
{
	if(val)
		NRF_GPIO->OUTSET = led_driver_mask;
	else
		NRF_GPIO->OUTCLR = led_driver_mask;
}

void leds_set(int r, int g, int b)
{
	if(r == 0)
	{
		NRF_GPIO->OUTSET = led_pin_mask[0];
		led_pin_mask[0] = 0;
	}
	else led_pin_mask[0] = 1<<led_pins[0];
	if(g == 0)
	{
		NRF_GPIO->OUTSET = led_pin_mask[1];
		led_pin_mask[1] = 0;
	}
	else led_pin_mask[1] = 1<<led_pins[1];
	if(b == 0)
	{
		NRF_GPIO->OUTSET = led_pin_mask[2];
		led_pin_mask[2] = 0;
	}
	else led_pin_mask[2] = 1<<led_pins[2];

	pending_color_change = 1;
	if(r != 0)
		pending_r = val_to_cc(r);
	else pending_r = 0xFFFF;
	if(g != 0)
		pending_g = val_to_cc(g);
	else pending_g = 0xFFFF;
	if(b != 0)
		pending_b = val_to_cc(b);
	else pending_b = 0xFFFF;
	return;
}

void leds_pulse(int r, int g, int b, int length)
{
	leds_set(r, g, b);
	led_pulse_length = 1 + length/32;
}

void leds_set_default_color(int r, int g, int b)
{
	led_default_r = r;
	led_default_g = g;
	led_default_b = b;
}
void leds_pulse_default(int length)
{
	leds_pulse(led_default_r, led_default_g, led_default_b, length);
}

void color_h2rgb(int h, uint8_t *r, uint8_t *g, uint8_t *b) //HSV color model with S=V=100
{
	uint8_t hi = (h/60)%6;
	uint8_t a = (h%60)*51/12; //a: 0...255
	uint8_t Vinc = a;
	uint8_t Vdec = 255 - a;

	switch(hi)
	{
		case 0: 
			*r = 255;
			*g = Vinc;
			*b = 0;
			break;
		case 1: 
			*r = Vdec;
			*g = 255;
			*b = 0;
			break;
		case 2: 
			*r = 0;
			*g = 255;
			*b = Vinc;
			break;
		case 3: 
			*r = 0;
			*g = Vdec;
			*b = 255;
			break;
		case 4: 
			*r = Vinc;
			*g = 0;
			*b = 255;
			break;
		case 5: 
			*r = 255;
			*g = 0;
			*b = Vdec;
			break;
		default:
			*r = 255;
			*g = 255;
			*b = 255;
	}
	return;
}
