/**
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "mcp3911.h"
//#include "bmi160.h"
#include "leds.h"
#include "ecg_processor.h"
#include "urf_timer.h"
#include "urf_radio.h"
#include "urf_ble_peripheral.h"
#include "ble_const.h"
#include "board_config.h"
#include "radio_functions.h"
#include "spim_functions.h"
#include "imu_processor.h"

//openocd -f interface/stlink-v2.cfg -f target/nrf52.cfg
//flash write_image erase _build/nrf52832_xxaa.hex

int battery_mv = 3500; //by default non zero to prevent power off before actual measurement
int battery_low_threshold = 3200;

void fast_clock_start()
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
}
void slow_clock_start()
{
	NRF_CLOCK->LFCLKSRC = 0;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {}
}
void fast_clock_stop()
{
	slow_clock_start();
	NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}

void switch_to_ble()
{
	rf_disable();
	ble_init_radio();
}

void switch_to_fr32()
{
	rf_disable();
//	fr_poweroff();
	rf_init(21, 1000, 0);
	rf_listen();	
}

void switch_to_fr64()
{
	rf_disable();
//	fr_poweroff();
	rf_init(21, 1000, 3);
	rf_listen();	
}

uint32_t prev_short_press_time = 0;
//int mcp_filter_state = 1;

void process_btn_short()
{
//	mcp_filter_state = !mcp_filter_state;
//	mcp_set_filter_mode(mcp_filter_state); //50/60 hz filter debug
	uint32_t ms = millis();
	if(ms - prev_short_press_time > 800)
	{
		prev_short_press_time = ms;
		return;
	}
	prev_short_press_time = 0; //double presses only
	if(dev_config.radio_mode == radio_mode_ble_adv || dev_config.radio_mode == radio_mode_ble_conn)
		dev_config.radio_mode = radio_mode_direct64;
	else if(dev_config.radio_mode == radio_mode_direct64)
		dev_config.radio_mode = radio_mode_direct32;
	else
		dev_config.radio_mode = radio_mode_ble_adv;
		
	radio_set_mode(dev_config.radio_mode);
	if(dev_config.radio_mode == radio_mode_ble_adv)
	{
		for(int x = 0; x < 3; x++)
		{
			leds_set(0, 0, 255);
			delay_ms(200);
			leds_set(0, 0, 0);
			delay_ms(200);
		}
	}
	if(dev_config.radio_mode == radio_mode_direct64)
	{
		for(int x = 0; x < 3; x++)
		{
			leds_set(0, 255, 0);
			delay_ms(200);
			leds_set(0, 0, 0);
			delay_ms(200);
		}
	}
	if(dev_config.radio_mode == radio_mode_direct32)
	{
		for(int x = 0; x < 3; x++)
		{
			leds_set(255, 0, 0);
			delay_ms(200);
			leds_set(0, 0, 0);
			delay_ms(200);
		}
	}
}

void process_btn_long()
{
	if(dev_config.signal_measurement_mode == signal_measurement_ecg)
	{
		dev_config.signal_measurement_mode = signal_measurement_emg;
		mcp_fft_mode(1);
	}
	else
	{
		dev_config.signal_measurement_mode = signal_measurement_ecg;
		mcp_fft_mode(0);		
	}
		
	if(dev_config.signal_measurement_mode == signal_measurement_ecg)
	{
		leds_set(0, 255, 255);
		delay_ms(300);
		leds_set(0, 0, 0);
		delay_ms(200);
		leds_set(0, 255, 255);
		delay_ms(300);
		leds_set(0, 0, 0);
		delay_ms(100);
	}
	else
	{
		leds_set(255, 255, 0);
		delay_ms(300);
		leds_set(0, 0, 0);
		delay_ms(200);
		leds_set(255, 255, 0);
		delay_ms(300);
		leds_set(0, 0, 0);
		delay_ms(100);
	}
}

int main(void)
{
	int init_ok = 1;
	
	fast_clock_start();
	time_start();
	board_config_init();
	device_config_init();
	board_power_on();
	board_analog_on();
	
	leds_init(board_config.led_r, board_config.led_g, board_config.led_b, board_config.out_driver);

	for(int x = 0; x < 3; x++)
	{
		leds_pulse((x==0)*255, (x==1)*255, (x==2)*255, 300);
		delay_ms(300);
	}
	if(0)for(int x = 0; x < 8; x++)
	{
		leds_pulse(0, 255*(board_config.version == 4), 255*(board_config.version == 5), 100);
		delay_ms(200);
		
//		leds_set(0, 255*(x%2)*(board_config.version == 4), 255*(x%2)*(board_config.version == 5));
//		delay_ms(100);
	}


	leds_set(0, 0, 0);
	delay_ms(300);
	
	spi_init();
	
	mcp3911_init(board_config.mcp_CS, board_config.mcp_DR);
	if(!mcp3911_is_ok())
	{
		init_ok = 0;
		leds_pulse(255, 0, 0, 1000);
		delay_ms(1200);
		leds_pulse(255, 0, 255, 1000);
		delay_ms(1200);
	}

	bmi160_init(board_config.bmi_CS, board_config.bmi_INT);
	if(!bmi160_is_ok())
	{
		init_ok = 0;
		leds_pulse(255, 0, 0, 1000);
		delay_ms(1200);
		leds_pulse(255, 255, 0, 1000);
		delay_ms(1200);
	}
//	leds_set(0, 255, 0);

	delay_ms(100);
	
	ecg_processor_init();

	radio_fill_ble_services();
	radio_set_mode(dev_config.radio_mode);

	if(init_ok)
	{
		for(int x = 0; x < 3; x++)
		{
			leds_pulse(0, 255, 0, 200);
			delay_ms(400);
//			leds_set(0, 255, 0);
//			leds_set(0, 0, 0);
//			delay_ms(200);
		}
	}

	dev_state.battery_mv = board_read_battery();
	
	NRF_WDT->CRV = 4*32768; //4 seconds timeout - impossible to miss during normal operation
	NRF_WDT->TASKS_START = 1;

	while(1)
	{
		NRF_WDT->RR[0] = 0x6E524635; //reload watchdog
		uint32_t ms = millis();
		
		
		if(board_read_button())
		{
			dev_state.led_overriden = 1;
			if(!dev_state.btn_pressed)
			{
				dev_state.btn_pressed = 1;
				dev_state.btn_on_time = ms;
			}
			if(ms - dev_state.btn_on_time > 25 && ms - dev_state.btn_on_time < 500)
				leds_set(0, 0, 255);
			if(ms - dev_state.btn_on_time > 500 && ms - dev_state.btn_on_time < 2000)
				leds_set(0, 255, 0);
			if(ms - dev_state.btn_on_time > 2000)
				leds_set(255, 0, 0);
		}
		else
		{
			dev_state.led_overriden = 0;
			if(dev_state.btn_pressed)
			{
				dev_state.btn_pressed = 0;
				uint32_t btn_time = ms - dev_state.btn_on_time;
				if(btn_time > 25) //ignore too short presses - noise
				{
					if(btn_time < 500)
						process_btn_short();
					else if(btn_time < 2000)
						process_btn_long();
					else
					{
						board_power_off();
					}
				}
			}
		}
		if(mcp3911_read())
		{
			process_mcp_data();
			//need to read BMI data right after MCP data are available so transfer won't be interrupted
			if(ms - bmi.imu_read_time > 10)
			{
				bmi160_read();
				process_imu_data(&bmi);
				bmi.T = 0.25f*NRF_TEMP->TEMP;
				NRF_TEMP->TASKS_START = 1;
				bmi.imu_read_time = ms;
				static int prev_bmi_ax = 0;
				static int prev_bmi_ay = 0;
				static int prev_bmi_az = 0;
				static int last_bmi_change = 0;
				if(prev_bmi_ax != bmi.raX || prev_bmi_ay != bmi.raY || prev_bmi_az != bmi.raZ) last_bmi_change = millis();
				prev_bmi_ax = bmi.raX;
				prev_bmi_ay = bmi.raY;
				prev_bmi_az = bmi.raZ;
				if(ms - last_bmi_change > 500) //impossible during normal operation: noise would be present anyway
				{
					static int in_bmi_reinit = 0;
					if(in_bmi_reinit)
					{
						if(bmi160_reinit_done()) in_bmi_reinit = 0;
						last_bmi_change = ms;
					}
					else if(!bmi160_is_ok())
					{
						in_bmi_reinit = 1;
						bmi160_reinit_start(); //reset settings
					}
				}
			}
			else if(ms - bmi.imu_read_time > 5 && ms - bmi.steps_read_time > 311)
			{
				bmi.steps_read_time = ms;
				bmi160_read_steps();
			}
			else if(ms - bmi.imu_read_time > 5 && ms - bmi.temp_read_time > 573)
			{
				bmi.temp_read_time = ms;
//				bmi160_read_temp();
			}
		}
		
		radio_process_sending();
		
		if(ms - dev_state.battery_update_time > 503)
		{
			dev_state.battery_mv = (7*dev_state.battery_mv + board_read_battery())/8;
			dev_state.battery_update_time = ms;
			if(dev_state.battery_mv < 3200 && ms > 10000) //too low battery - shutdown or LP mode
			{
				for(int x = 0; x < 20; x++)
				{
					leds_set(255, 0, 0);
					delay_ms(50);
					leds_set(0, 0, 0);
					delay_ms(50);
				}
				board_power_off();
			}
		}
	}

}

