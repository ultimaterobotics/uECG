#include "board_config.h"
#include "mcp3911.h"
#include "bmi160.h"
#include "leds.h"
#include "urf_timer.h"
#include "nrf.h"

sBoardConfig board_config;
sDeviceState dev_state;
sDeviceConfig dev_config;

void board_config_init()
{
	//in version 4, pin 22 is floating, in version 5 it is tied to ground
	board_config.version_id_pin = 22;
	sNRF_GPIO_config version_pin;
	version_pin.conf = 0;
	version_pin.is_output = 0;
	version_pin.pull = 0b11;
	NRF_GPIO->PIN_CNF[board_config.version_id_pin] = version_pin.conf;
	int cnt_up = 0;
	for(int x = 0; x < 10; x++)
		if(NRF_GPIO->IN & (1<<board_config.version_id_pin)) cnt_up++;
	if(cnt_up > 8) //high state -> old version
		board_config.version = 4;
	else
		board_config.version = 5; //only two possibilities as of now

	version_pin.pull = 0; //turn off pullup so current won't leak there for ver 5
	NRF_GPIO->PIN_CNF[board_config.version_id_pin] = version_pin.conf;
	
	board_config.unit_id = NRF_FICR->DEVICEID[1];
	
	if(board_config.version == 4)
	{
		board_config.led_r = 6;
		board_config.led_g = 7;
		board_config.led_b = 8;
		board_config.button = 26;
		board_config.bat_sense = 2;
		
		board_config.spi_COPI = 12;
		board_config.spi_CIPO = 13;
		board_config.spi_SCK = 14;
		
		board_config.mcp_CS = 15;
		board_config.mcp_FCLK = 16;
		board_config.mcp_DR = 17;
		
		board_config.bmi_CS = 18;
		board_config.bmi_INT = 20;
		
		board_config.amp_EN = 11; 
		//amplifier turn off doesn't properly function in ver 3, where it's located on pin 30, there it shouldn't be ever turned off
		
		board_config.out_driver = 19;
	}
	if(board_config.version == 5)
	{
		board_config.led_r = 6;
		board_config.led_g = 7;
		board_config.led_b = 8;
		board_config.button = 27;
		board_config.bat_sense = 2;
		
		board_config.spi_COPI = 12;
		board_config.spi_CIPO = 13;
		board_config.spi_SCK = 14;

		board_config.mcp_CS = 15;
		board_config.mcp_FCLK = 16;
		board_config.mcp_DR = 17;
		
		board_config.bmi_INT = 10;
		board_config.bmi_CS = 11;
		
		board_config.amp_EN = 18;
		board_config.sys_ON = 24;
		
		board_config.out_driver = 0xFF; //not present in version 5
	}

	sNRF_GPIO_config pin;
	pin.conf = 0;
	pin.is_output = 1;
	NRF_GPIO->PIN_CNF[board_config.amp_EN] = pin.conf;
	if(board_config.version == 4) //could be version 3 which can't be autodetected - in this case turn on amplifier at another pin
		NRF_GPIO->PIN_CNF[30] = pin.conf;
	NRF_GPIO->PIN_CNF[board_config.sys_ON] = pin.conf;
	NRF_GPIO->PIN_CNF[board_config.spi_COPI] = pin.conf;
	NRF_GPIO->PIN_CNF[board_config.spi_SCK] = pin.conf;
	pin.is_output = 0;
	if(board_config.version == 4)
		pin.pull = 0b11;
	else
		pin.pull = 0;
	NRF_GPIO->PIN_CNF[board_config.button] = pin.conf;
	pin.pull = 0;
	NRF_GPIO->PIN_CNF[board_config.spi_CIPO] = pin.conf;
}

void board_power_on()
{
	if(board_config.version == 4)
	{
		NRF_GPIO->OUTSET = (1<<board_config.amp_EN);
		NRF_GPIO->OUTSET = (1<<30); //in case if it's actually version 3 which wasn't detected
	}
	if(board_config.version == 5)
	{
		NRF_GPIO->OUTSET = (1<<board_config.sys_ON) | (1<<board_config.amp_EN);
		NRF_POWER->DCDCEN = 1;
	}
}
void board_power_off()
{
	//in v5, just turn off everything
	if(board_config.version == 5)
		NRF_GPIO->OUTCLR = (1<<board_config.sys_ON) | (1<<board_config.amp_EN);
	
	//in v4, there is no such option - instead, get everything into lower power consumption mode
	if(board_config.version == 4)
	{
		mcp3911_turnoff();
		bmi160_stop();
		leds_set(0, 0, 0);
		leds_set_driver(0);
		delay_ms(10); //to make sure led state was applied - it's async
		time_stop();
		NRF_SPI0->ENABLE = 0;
		NRF_GPIO->OUTCLR = (1<<board_config.amp_EN);
		NRF_CLOCK->TASKS_HFCLKSTOP = 1;
		NRF_POWER->TASKS_LOWPWR = 1;
		NRF_POWER->SYSTEMOFF = 1;
	}
}

//we may turn analog part on/off in order to reduce power consumption
//if we take readings not all the time
void board_analog_on()
{
	NRF_GPIO->OUTSET = (1<<board_config.amp_EN);
	mcp_start_clock();
	board_config.analog_is_on = 1;
}
void board_analog_off()
{
	NRF_GPIO->OUTCLR = 1<<board_config.amp_EN;
	mcp_stop_clock();
	board_config.analog_is_on = 0;
}

int board_read_button()
{
	if(board_config.version == 4)
		return (NRF_GPIO->IN & (1<<board_config.button)) == 0;
	
	return (NRF_GPIO->IN & (1<<board_config.button)) > 0;
}

int board_read_battery()
{
	uint32_t result = 0;
	
	// Configure SAADC singled-ended channel, Internal reference (0.6V) and 1/6 gain.
	//15 us sampling time required for our battery resistor network (although probably less would work
	//fine since there is a capacitor there as well)
	NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
							(SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos) |
							(SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
							(SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
							(SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
							(SAADC_CH_CONFIG_TACQ_15us        << SAADC_CH_CONFIG_TACQ_Pos);

	// Configure the SAADC channel with VDD as positive input, no negative input(single ended).
	int ain_code = 0;
	if(board_config.bat_sense >= 2 && board_config.bat_sense <= 5) ain_code = 1 + board_config.bat_sense - 2;
	if(board_config.bat_sense >= 28 && board_config.bat_sense <= 32) ain_code = 5 + board_config.bat_sense - 28;
	
	NRF_SAADC->CH[0].PSELP = ain_code;
	NRF_SAADC->CH[0].PSELN = 0;

	// Configure the SAADC resolution.
	NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;

	// Configure result to be put in RAM at the location of "result" variable.
	NRF_SAADC->RESULT.MAXCNT = 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)&result;

	// No automatic sampling, will trigger with TASKS_SAMPLE.
	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

	// Enable SAADC (would capture analog pins if they were used in CH[0].PSELP)
	NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

/*	if(!adc_calibrated) //no real need for high precision here
	{
		// Calibrate the SAADC (only needs to be done once in a while)
		NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
		while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
		NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
		while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy <<SAADC_STATUS_STATUS_Pos));
		adc_calibrated = 1;
	}*/

	// Start the SAADC and wait for the started event.
	NRF_SAADC->TASKS_START = 1;
	while (NRF_SAADC->EVENTS_STARTED == 0);
	NRF_SAADC->EVENTS_STARTED = 0;

	// Do a SAADC sample, will put the result in the configured RAM buffer.
	NRF_SAADC->TASKS_SAMPLE = 1;
	while (NRF_SAADC->EVENTS_END == 0);
	NRF_SAADC->EVENTS_END = 0;

	// Stop the SAADC, since it's not used anymore.
	NRF_SAADC->TASKS_STOP = 1;
	while (NRF_SAADC->EVENTS_STOPPED == 0);
	NRF_SAADC->EVENTS_STOPPED = 0;
	
	float res = result;
	res = res * 3600.0 / 4095.0; //600mV reference with 1/6 divider means 3600 is equal to 4095
	res *= 3.0; //1:3 divider of the battery measurement circuit
	int rres_mv = res + 0.5; //proper rounding
	return rres_mv; //in mV
}

//reads stored config (or fills default values if nothing is stored)
//and places it in both current and saved configuration structures,
//current config may be modified without storing, modifications of
//stored config have no effect - only device_config_store function actually
//stores changes on flash memory
void device_config_init()
{
	if(1) //proper implementation pending - flash memory should be read and
	{ //those values filled only if it doesn't contain proper configuration
		dev_config.validation_word = 0xAA0EC90A;
		dev_config.ecg_advertising_enabled = 0;
		dev_config.led_color_b = 250;
		dev_config.led_color_g = 80;
		dev_config.led_color_r = 220;
		dev_config.led_enabled = 1;
		dev_config.gyro_enabled = 0;
		dev_config.power_mode = device_power_normal;
		dev_config.radio_mode = radio_mode_ble_adv;
		dev_config.signal_measurement_mode = signal_measurement_ecg;
	}
}

//stores provided config
void device_config_store(sDeviceConfig *config)
{
	//implementation pending
}
//(de)serialization is intended to be used also in Android app and possibly in PC apps,
//so we can't rely on memory layout, instead explicit transformation is required
void device_config_serialize(sDeviceConfig *config, uint8_t buf)
{
	//implementation pending
}
void device_config_deserialize(sDeviceConfig *config, uint8_t buf)
{
	//implementation pending
}
