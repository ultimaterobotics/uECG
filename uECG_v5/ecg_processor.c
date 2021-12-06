#include "board_config.h"
#include "ecg_processor.h"
#include "leds.h"
#include "urf_timer.h"
#include "mcp3911.h"

sRdetector r_detector;
sHRVparams hrv_params;
sEMGparams emg_params;
sECGparams ecg_params;

//we keep a lot of RR intervals in memory, just in case if more advanced processing will be required later
int rr_hist_len = 2500;
int rr_hist_pos = 0;
uint16_t RR_hist[2500];

int emg_mode = 0;

//insertion sort for small arrays - efficient for small ones only
void sort_insert(int *arr, int len)
{
    int i, key, j;
    for(i = 1; i < len; i++)
    {
		key = arr[i];
		j = i - 1;
		while (j >= 0 && arr[j] > key)
		{
			arr[j+1] = arr[j];
			j--;
		}
		arr[j + 1] = key;
    }	
}

//when new RR interval is added, recalculate current BPM
void update_BPM()
{
	int beats = 25; //BPM will be calculated over that many last beats
	int arr_RR[25]; //need to be of beats length

	int beats_momentary = 5; //in some cases we want to know immediate BPM value, for that 
	int arr_RR_m[10]; //purpose only a few beats are used
	
	int hpos = rr_hist_pos - 1;
	for(int x = 0; x < beats; x++)
	{
		if(hpos < 0) hpos += rr_hist_len;
		arr_RR[x] = RR_hist[hpos];
		if(x < beats_momentary)
			arr_RR_m[x] = RR_hist[hpos];
		hpos--;
	}
	//sort beats by value, so we can easily exclude longest and shortest
	//in case of heavy noise (like while running) it greatly improves stability
	//and makes virtually no difference in case of low noise
	sort_insert(arr_RR, beats);
	sort_insert(arr_RR_m, beats_momentary);
	
	float avg_rr = 0;
	int exclude_cnt = 6; //don't count shortest and longes beats, kinda median filter
	for(int x = exclude_cnt; x < beats-exclude_cnt; x++)
		avg_rr += arr_RR[x];
	avg_rr /= (beats - exclude_cnt*2); //in milliseconds
	if(avg_rr < 1) avg_rr = 100000; //when turned on, there are no data - using impossible value to indicate that
	ecg_params.BPM_normal = 60.0 * (1000.0 / avg_rr);
	
	avg_rr = 0;
	for(int x = 1; x < beats_momentary-1; x++)
		avg_rr += arr_RR_m[x];
	avg_rr /= (beats_momentary - 2); //in milliseconds
	if(avg_rr < 1) avg_rr = 100000; //when turned on, there are no data - using impossible value to indicate that
	ecg_params.BPM_momentary = 60.0 * (1000.0 / avg_rr);
	return;
}

void hrv_params_init()
{
	hrv_params.avg_speed = 0.01;
	hrv_params.sdrr2 = 0.0;
	hrv_params.sdrr = 0;
	hrv_params.rmssd = 0;
	hrv_params.pNN_bins = 15;
	for(int x = 0; x < hrv_params.pNN_bins; x++)
	{
		hrv_params.pNN[x] = 0;
		hrv_params.pNN_norm[x] = 0;
	}
	hrv_params.pnn_avg_speed = 0.01;
}

//when new R peak is detected, we also update HRV parameters
//standard approach involves calculation over fixed periods of time,
//but this is computationally more expensive and benefits of that approach
//are questionable - so we implemented a different one
void update_HRV()
{
	//take two most recent RR intervals
	int hpos = rr_hist_pos - 1;
	if(hpos < 0) hpos += rr_hist_len;
	float rr = RR_hist[hpos];
	hpos--;
	if(hpos < 0) hpos += rr_hist_len;
	float rr2 = RR_hist[hpos];
	
	if(rr < 200 || rr > 2000 || rr2 < 200 || rr2 > 2000) return; //bad RRs, no update
	
	//calculate STD
	float d_rr = rr2 - rr;
	hrv_params.sdrr2 *= (1.0 - hrv_params.avg_speed);
	hrv_params.sdrr2 += hrv_params.avg_speed * d_rr*d_rr;
	hrv_params.sdrr = sqrt_fast(hrv_params.sdrr2);
	
	//transfer current deviation into percentage of current RR interval
	if(ecg_params.BPM_normal < 1) return; //not enough data yet
	if(d_rr < 0) d_rr = -d_rr;
	float cur_avg_RR = 60000.0 / (float)ecg_params.BPM_normal;
	//each bin represents 1% variation. Bin 0 means that variation is <1%,
	//bin 1 means that variation is from 1 to 2% etc,
	//the last bin has sum of all larger variations
	int dRR_bin = d_rr / (0.01 * cur_avg_RR);
	if(dRR_bin >= hrv_params.pNN_bins) dRR_bin = hrv_params.pNN_bins-1;

	//averaging all bins
	for(int x = 0; x < hrv_params.pNN_bins; x++)
		hrv_params.pNN[x] *= 1.0 - hrv_params.pnn_avg_speed;
	hrv_params.pNN[dRR_bin] += 1.0;

	//and fill normalized bins (more convenient than absolute values)
	float sum = 0.000001;
	for(int x = 0; x < hrv_params.pNN_bins; x++)
		sum += hrv_params.pNN[x];
	float msum = 100.0 / sum;
	for(int x = 0; x < hrv_params.pNN_bins; x++)
		hrv_params.pNN_norm[x] = hrv_params.pNN[x] * msum;

	//now we need 3rd RR interval for rmssd calculations
	hpos--;
	if(hpos < 0) hpos += rr_hist_len;
	float rr3 = RR_hist[hpos];

	if(rr3 > 200 && rr3 < 2000) //if it's good as well
	{
		d_rr = rr2 - rr; //to restore sign which was lost in previous section
		float d_rr2 = rr3 - rr2;
		float dif_rr = d_rr - d_rr2; //STD of 2nd order
		hrv_params.rmssd2 *= 1.0 - hrv_params.avg_speed;
		hrv_params.rmssd2 += hrv_params.avg_speed * dif_rr*dif_rr;
		hrv_params.rmssd = sqrt_fast(hrv_params.rmssd2);
	}
}

//add new RR interval into array and call corresponding update functions
void push_RR(uint32_t RR)
{
	RR_hist[rr_hist_pos] = RR;
	rr_hist_pos++;
	if(rr_hist_pos >= rr_hist_len)
		rr_hist_pos = 0;
	
	ecg_params.rr_id++;
	ecg_params.unsent_RR_cnt++;
	
	update_BPM();
	update_HRV();
}

void r_detector_init()
{
	r_detector.avg_dv_p = 0;
	r_detector.avg_dv_n = 0;
	r_detector.dv_p_peak = 0;
	r_detector.dv_n_peak = 0;
	r_detector.dv_p_peak_time = 0;
	r_detector.dv_n_peak_time = 0;
	r_detector.R_time = 0;
	r_detector.R_detected = 0;

	r_detector.v_dec_avg = 0;
	r_detector.v_dec = 0;
	r_detector.dec_l = 8;
	r_detector.dec_p = 0;
}

//R peak detector is applied every time new ECG value is measured
//due to high data rate, R detector uses not many calculations so it can
//run in realtime without consuming too much processor resources
void r_detector_step(float v)
{
	//calculate short average and speed of signal change
	float prev_s = r_detector.avg_s;
	r_detector.avg_s *= 0.9;
	r_detector.avg_s += 0.1*v;
	
	float dv = r_detector.avg_s - prev_s;
	//fast average of current speed of signal change in positive and negative directions
	r_detector.avg_dv_p *= 0.95;
	r_detector.avg_dv_n *= 0.95;
	if(dv > 0)
		r_detector.avg_dv_p += 0.05 * dv;
	else
		r_detector.avg_dv_n -= 0.05 * dv; //minus sign because dv is negative but we want positive value

	//slowly reduce peak value over time, so that next R beat definitely will be higher than previous
	//absolute peak value isn't very stable, but with this method it works well
	r_detector.dv_p_peak *= 0.999;
	r_detector.dv_n_peak *= 0.999;
	
	//by default assuming we are not in R peak, but that might change in following lines
	r_detector.R_detected = 0;
	
	//each R peak consists of high increase in positive and negative signal changes, but we 
	//don't know their order in advance - it depends on sensor placement. So criteria is based
	//on detecting peak in one direction, and if opposite direction had a peak just before that - 
	//then it's our R peak
	if(r_detector.avg_dv_p > r_detector.dv_p_peak)
	{
		//update positive peak
		r_detector.dv_p_peak_time = 0;
		r_detector.dv_p_peak = r_detector.avg_dv_p;
		r_detector.p_peak_v = v;
		if(r_detector.dv_n_peak_time < 70) //and if negative was just before - it's R peak
//		if(r_detector.dv_n_peak_time < 30) //lemur variant
		{
			r_detector.R_detected = 1;
			r_detector.R_time = 0;
		}
	}
	if(r_detector.avg_dv_n > r_detector.dv_n_peak)
	{
		//update negative peak
		r_detector.dv_n_peak_time = 0;
		r_detector.dv_n_peak = r_detector.avg_dv_n;
		r_detector.n_peak_v = v;
		if(r_detector.dv_p_peak_time < 70) //and if positive was just before - it's R peak
//		if(r_detector.dv_p_peak_time < 30) //lemur variant
		{
			r_detector.R_detected = 1;
			r_detector.R_time = 0;
		}
	}
	
	if(r_detector.R_time == 30) //~30 milliseconds after R peak - long enough 
	{ //to make sure detected point is stable, so we can push new RR interval
		uint32_t ms = millis();
		if(ms - r_detector.cur_peak_time > 150) //detector is not perfect, so need to ignore multiple detections
//		if(ms - r_detector.cur_peak_time > 60) //lemur variant
		{
			r_detector.prev_peak_time = r_detector.cur_peak_time;
			r_detector.cur_peak_time = ms;
			int RR = r_detector.cur_peak_time - r_detector.prev_peak_time;
			if(RR < 2000) //if RR is too long - it means that we missed some previous R peak(s),
				push_RR(RR); //so we won't be adding RR if that was the case
			
			//indicate detected beat with led
			if(ecg_params.led_enabled)
				leds_pulse_default(100);
//				leds_pulse_default(32); //lemur variant
		}
	}
	
	//these times are in ADC data points, not milliseconds!
	//that is used only to indicate that some time has passed
	r_detector.dv_n_peak_time++;
	r_detector.dv_p_peak_time++;
	r_detector.R_time++;
	
	//that part is not exactly R peak processing, it is averaging of data for sending via BLE
	//at reduced data rate. But when we have an R peak, average value is not good enough: peak is
	//too fast, and we won't always see actual peak value on averaged stream. So when we are in
	//peak state, we use peak value instead of average value - that's why processing is
	//added into this function
	r_detector.dec_p++;
	r_detector.v_dec_avg += v;
	if(r_detector.dec_p >= r_detector.dec_l)
	{
		r_detector.dec_p = 0;
		r_detector.v_dec = r_detector.v_dec_avg / r_detector.dec_l; //normally use average value
		r_detector.v_dec_avg = 0;
		//but if peak was within our averaging interval, use it instead
		//that somewhat deforms ECG shape around the peak, but since low frequency
		//data stream isn't good for R shape analysis anyway, we chose to preserve
		//R peak amplitude instead of its shape
		if(r_detector.dv_n_peak_time <= r_detector.dec_l)
			r_detector.v_dec = r_detector.n_peak_v;
		if(r_detector.dv_p_peak_time <= r_detector.dec_l)
			r_detector.v_dec = r_detector.p_peak_v;
	}
}

//You may ask why EMG processing is in "ecg_processor" file. That's a good question.
//Mostly because it is a side function of uECG device, and it does almost nothing with
//EMG - only calculates its spectrum and blinks leds accordingly
int process_mcp_data_emg()
{
	//all processing is based on fft, which is calculated inside mcp3911 processing cycle,
	//together with basic filtering - so when it's not ready, we do nothing
	if(!mcp_fft_process()) return 0;

	float *sp = mcp_fft_get_spectr();
	float norm = 0.001;
	float avg_center = 0;
	float low_part = 0;
	float high_part = 0;

	//initially spectrum was longer and divided into low, central and high frequencies
	//but it turned out that 4 points are enough and it simplifies things a lot
	//still, names remain - although now high and central are the same region
	for(int x = 0; x < 4; x++)
	{
		if(sp[x] > norm) norm = sp[x];
		if(x > 0) avg_center += sp[x];
		if(x == 0) low_part += sp[x];
		else high_part += sp[x];
	}
	high_part *= 0.33333; //scale
	
	float h_coeff = 2.0*high_part / (low_part + high_part + 0.001); //0.001 term to avoid division by zero

	//non-linear: muscle signal is concentrated in higher frequency area so we
	//are not interested in low frequency signal even if it's strong
	avg_center *= 0.03 * h_coeff;
	
	emg_params.value *= 0.8;
	emg_params.value += 0.2*avg_center;
	
	//show muscle activity level as varying color/intensity of LED
	if(emg_params.led_enabled)
	{
		float thr_0 = 5.0; //zero until here
		float thr_1 = 15.0; //1st color until here
		float thr_2 = 30.0; //2nd color blend in
		float thr_3 = 45.0; //2nd color blend out, 3rd in, past threshold - max color
		if(emg_params.value < thr_0)
			leds_set(0, 0, 0);
		else if(emg_params.value < thr_1)
		{
			float vv = (emg_params.value-thr_0) / (thr_1 - thr_0);
//			leds_set(0, 0, 255.0*vv);
			leds_pulse(0, 0, 255.0*vv, 10);
		}
		else if(emg_params.value < thr_2)
		{
			float vv = (emg_params.value-thr_1) / (thr_2 - thr_1);
//			leds_set(255.0*vv, 0, 255.0);
			leds_pulse(255.0*vv, 0, 255.0, 10);
		}
		else if(emg_params.value < thr_3)
		{
			float vv = (emg_params.value-thr_2) / (thr_3 - thr_2);
//			leds_set(255.0*(1.0-vv), 255*vv, 255);
			leds_pulse(255.0*(1.0-vv), 255*vv, 255, 10);
		}
		else
		{
//			leds_set(0, 255, 255);
			leds_pulse(0, 255, 255, 10);
		}
	}
	//level as uint16 for simple radio transmission
	emg_params.level = emg_params.value * 100.0;
	if(emg_params.value > 650) emg_params.level = 65000;
	
	//normalized spectral values, already in uint8 buffer for transmission
	emg_params.spectr_scale = norm;
	norm = 32760.0 / norm;
	for(int x = 0; x < 4; x++)
	{
		int16_t vv = sp[x] * norm;
		emg_params.spectr_buf[x*2] = (vv>>8)&0xFF;
		emg_params.spectr_buf[x*2+1] = vv&0xFF;
	}
	emg_params.data_id++;
	return 1;
}

//main processing function that should be called each time ADC has new data point ready,
//calls everything else according to current settings
int process_mcp_data()
{
	if(dev_config.signal_measurement_mode == signal_measurement_emg) return process_mcp_data_emg();
	
	//get new data point and store it in circular buffer
	int filtered_value = mcp_get_filtered_value();
	ecg_params.data_buffer[ecg_params.buf_pos++] = filtered_value;
	if(ecg_params.buf_pos >= ecg_params.buf_len) ecg_params.buf_pos = 0;
	ecg_params.data_id++;
	ecg_params.unsent_data_cnt++;
	ecg_params.skin_parameter = mcp_get_filtered_skin();

	//R peak detector, it also takes care of registering RR intervals
	r_detector_step(filtered_value);
	
	//once per decimation cycle store value for BLE transmission at 1/8 of original frequency
	if(r_detector.dec_p == 0)
	{
		//also store it in a circular buffer, a different one for convenience - even though
		//we never really need both high and low frequency buffers at the same time,
		//but they don't take that much memory to care about that
		ecg_params.data_buffer_ble[ecg_params.ble_buf_pos++] = r_detector.v_dec;
		if(ecg_params.ble_buf_pos >= ecg_params.ble_buf_len) ecg_params.ble_buf_pos = 0;
		ecg_params.ble_data_id++;
	}
	return 1;
}

//get RR interval some steps back, 1 for most recent, 2 for 2nd recent etc, undefined at 0 
int get_RR(int hist_depth)
{
	int hpos = rr_hist_pos-hist_depth;
	while(hpos < 0) hpos += rr_hist_len;
	return RR_hist[hpos];
}

void set_emg_mode(int use_emg_mode)
{
	ecg_params.emg_mode = use_emg_mode;
}
void set_led_indication(int use_leds)
{
	ecg_params.led_enabled = use_leds;
	emg_params.led_enabled = use_leds;
}

sRdetector *get_r_detector()
{
	return &r_detector;
}
sHRVparams *get_hrv_params()
{
	return &hrv_params;
}
sEMGparams *get_emg_params()
{
	return &emg_params;
}
sECGparams *get_ecg_params()
{
	return &ecg_params;
}


void ecg_processor_init()
{
	hrv_params_init();
	r_detector_init();
	for(int x = 0; x < rr_hist_len; x++)
		RR_hist[x] = 0;
	
	ecg_params.emg_mode = 0;
	ecg_params.BPM_momentary = 0;
	ecg_params.BPM_normal = 0;
	ecg_params.rr_id = 0;

	ecg_params.data_id = 0;
	ecg_params.buf_len = 64;
	ecg_params.buf_pos = 0;

	ecg_params.ble_data_id = 0;
	ecg_params.ble_buf_len = 64;
	ecg_params.ble_buf_pos = 0;
	ecg_params.led_enabled = 1;

	
	emg_params.data_id = 0;
	emg_params.level = 0;
	emg_params.value = 0;
	emg_params.led_enabled = 1;
}
