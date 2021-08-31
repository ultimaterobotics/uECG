#include "nrf.h"

void leds_init(int pin_r, int pin_g, int pin_b, int pin_driver);
void leds_set(int r, int g, int b); //0-255, -1 -> don't change
void leds_set_driver(int val);
void leds_pulse(int r, int g, int b, int length);
void leds_set_default_color(int r, int g, int b);
void leds_pulse_default(int length);

void color_h2rgb(int h, uint8_t *r, uint8_t *g, uint8_t *b);