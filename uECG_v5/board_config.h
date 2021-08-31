#include <stdint.h>

//some pin numbers are not the same through board versions
//so they are defined as variables inside config struct
//instead of static defines

typedef struct sBoardConfig
{
	uint8_t version; //first supported is 4 (corresponds to PCB revision 4.2),
	//next version is 5 (corresponds to PCB revision 4.5), versions in between don't require any
	//config changes
	uint32_t unit_id;
	uint8_t version_id_pin;
	uint8_t led_r;
	uint8_t led_g;
	uint8_t led_b;
	uint8_t button;
	uint8_t bat_sense;
	
	uint8_t spi_CIPO;
	uint8_t spi_COPI;
	uint8_t spi_SCK;
	uint8_t mcp_CS;
	uint8_t mcp_DR;	
	uint8_t mcp_FCLK;
	uint8_t bmi_CS;
	uint8_t bmi_INT;

	uint8_t amp_EN;
	uint8_t sys_ON;
	
	uint8_t out_driver; //output optocouple - present only in v4
	
	uint8_t analog_is_on;
}sBoardConfig;

enum {
	device_power_normal = 0,
	device_power_10sec_50sec
};
enum {
	signal_measurement_ecg = 0,
	signal_measurement_emg
};
enum {
	radio_mode_direct64 = 0,
	radio_mode_direct32,
	radio_mode_ble_adv,
	radio_mode_ble_conn,
	radio_mode_off
};

typedef struct sDeviceConfig
{
	uint32_t validation_word; //something wrong can be written in the config memory location
	// - use this to ensure that config is valid (or fill default values if it's not)
	uint8_t led_enabled;
	uint8_t ecg_advertising_enabled;
	uint8_t led_color_r;
	uint8_t led_color_g;
	uint8_t led_color_b;
	uint8_t gyro_enabled;
	uint8_t radio_mode;
	uint8_t signal_measurement_mode;
	uint8_t power_mode;
}sDeviceConfig;

//for storing various stuff required to be shared between different
//functional parts
typedef struct sDeviceState
{
	uint16_t battery_mv;
	uint32_t battery_update_time;
	uint8_t led_overriden; //if we need some indication, make sure that regular stuff don't interfere
	uint8_t btn_pressed;
	uint32_t btn_on_time;
}sDeviceState;

typedef struct {
    union {
        struct {
			unsigned is_output : 1;
			unsigned in_disable : 1;
			unsigned pull : 2;
			unsigned : 4;
			unsigned drive : 3;
			unsigned : 5;
			unsigned sense : 2;
        };
        uint32_t conf;
    };
}sNRF_GPIO_config;

extern sBoardConfig board_config;
extern sDeviceState dev_state;
extern sDeviceConfig dev_config;

void board_config_init();
int board_read_button();
int board_read_battery(); //blocking call, takes ~20uS
void board_power_on(); //turn on both system power and analog supply in v5
void board_power_off();

void board_analog_on();
void board_analog_off();

void device_config_init();
void device_config_store(sDeviceConfig *config);
void device_config_serialize(sDeviceConfig *config, uint8_t buf);
void device_config_deserialize(sDeviceConfig *config, uint8_t buf);
