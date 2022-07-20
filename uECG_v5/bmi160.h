#include <stdint.h>

typedef struct sBMI160
{
	uint32_t CS;
	float rawA2SI;
	float rawG2SI;
	
	float aX;
	float aY;
	float aZ;

	float wX;
	float wY;
	float wZ;
	
	int16_t raX;
	int16_t raY;
	int16_t raZ;
	
	int16_t rwX;
	int16_t rwY;
	int16_t rwZ;
	
	uint16_t step_cnt;
	
	uint32_t data_id;
	
	uint8_t raw_data[12];

	float G;
	
	float T;
	
	float qW;
	float qX;
	float qY;
	float qZ;
	
	uint32_t imu_read_time;
	uint32_t temp_read_time;
	uint32_t steps_read_time;
}sBMI160;

extern sBMI160 bmi;

void bmi160_init(uint8_t pin_CS, uint8_t pin_INT);
//void bmi160_init(uint8_t pin_CIPO, uint8_t pin_COPI, uint8_t pin_SCK, uint8_t pin_CS, uint8_t pin_INT);
int bmi160_is_ok();
uint8_t bmi160_read();
void bmi160_enable_dtap();
void bmi160_stop();
void bmi160_resume();


int bmi_get_dtap();

void bmi160_normal_mode();
void bmi160_acc_mode();
void bmi160_lp_mode();
int bmi_get_tap();

void bmi160_reinit_start();
int bmi160_reinit_done();

void bmi160_read_steps();
float bmi160_read_temp();

sBMI160 *bmi160_get_params();

int get_cur_len();
int get_cur_ax(int p);
int get_cur_ay(int p);
int get_cur_az(int p);
