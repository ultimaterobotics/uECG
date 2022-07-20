#include <stdint.h>

typedef struct sECGparams
{
	uint8_t emg_mode;
	int BPM_normal;
	int BPM_momentary;
	uint32_t rr_id;
	uint32_t data_id;
	uint32_t ble_data_id;
	
	int buf_len;
	int buf_pos;
	int data_buffer[64];

	int ble_buf_pos;
	int ble_buf_len;
	int data_buffer_ble[64];
	
	int skin_parameter;
	
	uint8_t led_enabled;
	
	uint32_t unsent_RR_cnt;
	uint32_t unsent_data_cnt;
}sECGparams;

typedef struct sEMGparams
{
	float value; //strength of muscle signal
	uint16_t level; //scaled and limited value
	uint16_t spectr_scale;
	uint8_t spectr_buf[8];
	uint32_t data_id;
	uint8_t led_enabled;
}sEMGparams;

typedef struct sHRVparams
{
	float avg_speed; //averaging speed
	float sdrr2;
	float sdrr;
	float rmssd2;
	float rmssd;
	int pNN_bins; //must be not more than following arrays size
	float pNN[16];
	uint8_t pNN_norm[16];
	float pnn_avg_speed;
}sHRVparams;


typedef struct sRdetector
{
	float avg_s;
	float avg_l;
	float avg_dv_p;
	float avg_dv_n;
	float dv_p_peak;
	float dv_n_peak;
	float p_peak_vraw;
	float n_peak_vraw;
	float p_peak_v;
	float n_peak_v;
	
	//WARNING: all times here are in ADC steps, which runs at 976 Hz - correction
	//is required if they are translated into milliseconds
	int dv_p_peak_time;
	int dv_n_peak_time;
	int R_time;
	int R_detected;

	float v_dec_avg;
	float v_dec;
	int dec_l;
	int dec_p;
	
	//those times are in milliseconds
	uint32_t prev_peak_time;
	uint32_t cur_peak_time;
}sRdetector;

extern sRdetector r_detector;
extern sHRVparams hrv_params;
extern sEMGparams emg_params;
extern sECGparams ecg_params;

int get_RR(int hist_depth);
int process_mcp_data();
void ecg_processor_init();
void set_emg_mode(int use_emg_mode);
void set_led_indication(int use_leds);
