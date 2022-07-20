#include "nrf.h"
#include "data_packing.h"
#include "board_config.h"
#include "ecg_processor.h"
#include "bmi160.h"
#include "urf_ble_peripheral.h"
#include "urf_timer.h"

uint8_t packet_id = 0;
uint8_t data_packet[128];

uint8_t data_ecg_ch[96];
uint8_t data_hrv_ch[24];
uint8_t data_rr_ch[24];
uint8_t data_imu_ch[24];

uint8_t param_send_id = 0;
int bin_send_id = 0;

enum param_sends
{
	param_batt_bpm = 0,
	param_sdnn,
	param_skin_res,
	param_lastRR,
	param_imu_acc,
	param_imu_gyro,
	param_imu_steps,
	param_pnn_bins,
	param_end,
	param_emg_spectrum
};


float encode_acc(float acc)
{
	float shift = 0, dA = 0, coeff = 1;
	float am = acc;
	if(am < 0) am = -am;
	if(am < 2)
	{
		shift = 0;
		dA = 0;
		coeff = 25.0;
	}
	else if (am < 12)
	{
		shift = 50;
		dA = 2.0;
		coeff = 5.0;
	}
	else 
	{
		shift = 100;
		dA = 12.0;
		coeff = 2.0;
	}
	float res = shift + (am - dA) * coeff;
	if(acc < 0) res = -res;
	return res;
}


uint8_t encode_16_8(int val) //encodes signed 16 bit int into unsigned 8 bit with progressive scaling
{
	int vp = val;
	if(vp < 0) vp = -vp;
	if(vp > 32767) vp = 32767;
	vp>>=1;
	int pow4 = 0;
	while(vp >= (1<<(pow4*2))) pow4++;
	int div = 1<<(pow4*2);
	float res = (float)vp / (float)div; //between 0.25 and 1.0, stricly less than 1
	res -= 0.25; //now between 0 and 0.75
	if(res < 0) res = 0; //for zero case
	//and now we have 4 bits to encode res, 3 bits go for pow2 and one for sign
	int rr = res*20.0f + 0.5f;
	if(rr > 15) rr = 15; //precaution, in fact never should be
	if(val < 0) pow4 |= 0b1000;
	return (pow4<<4) | rr;
}
int decode_8_16(uint8_t val) //decodes 8-bit compressed value into signed 16 bit
{
	int pow4 = (val>>4)&0b111;
	int is_negative = val>>7;
	int rr = val&0xF;
	float r1 = 1<<(pow4*2);
	float r2 = rr;
	return 2*r1 * (0.25f + r2/20.0f) * (1 - 2*is_negative);
}

//intended for sending data to Arduino with RF24 module connected,
//thus no transmission control and reduced ECG data rate
int data_prepare_packet_r32()
{
	uint8_t idx = 0;
	packet_id++;
	data_packet[idx++] = packet_id;
	data_packet[idx++] = 32; //fill actual value in the end
	data_packet[idx++] = (board_config.unit_id>>24)&0xFF;
	data_packet[idx++] = (board_config.unit_id>>16)&0xFF;
	data_packet[idx++] = (board_config.unit_id>>8)&0xFF;
	data_packet[idx++] = (board_config.unit_id)&0xFF;
	uint8_t send_cnt = 9; //max number of data points that fit in 32 bytes with everything else
	//in case of EMG mode, prepare a different packet
	if(dev_config.signal_measurement_mode == signal_measurement_emg)
	{
		data_packet[idx++] = 218; //indicate that this is EMG packet with magic number
		data_packet[idx++] = param_emg_spectrum;
		data_packet[idx++] = (dev_state.battery_mv - 2000)/10;
		data_packet[idx++] = emg_params.data_id;
		data_packet[idx++] = emg_params.level>>8;
		data_packet[idx++] = emg_params.level;
		data_packet[idx++] = emg_params.spectr_scale>>8;
		data_packet[idx++] = emg_params.spectr_scale;
		for(int n = 0; n < 8; n++)
			data_packet[idx++] = emg_params.spectr_buf[n];
		data_packet[1] = idx+1;
		uint8_t check = 0;
		for(int x = 0; x < idx; x++)
			check += data_packet[x];
		data_packet[idx++] = check;
		return idx;	
	}
	//we are sending always the same number of data points, hoping that even if some packets 
	//are lost, they still overlap enough to provide an uninterrupted data stream.
	//with this value, receiver knows what data points are in the packet.
	//we use ble_data_id because arduino can't really handle high frequency data stream,
	//and ble stream goes at 1/8 frequency
	data_packet[idx++] = ecg_params.ble_data_id;
	
	//4 bytes are dedicated to various parameters that don't change fast, so we 
	//cycle through them and send values correspondingly
	int cur_send_id;
	param_send_id++;
	if(param_send_id >= param_end) param_send_id = 0;
	cur_send_id = param_send_id;

	data_packet[idx++] = cur_send_id; //indicate what we are sending
	if(cur_send_id == param_batt_bpm)
	{
		data_packet[idx++] = (dev_state.battery_mv-2000)/10;
		data_packet[idx++] = 6; //protocol version_id - it's not the same as device version id
		data_packet[idx++] = ecg_params.BPM_normal;
	}
	if(cur_send_id == param_sdnn)
	{
		int v1 = hrv_params.sdrr;
		int v2 = hrv_params.rmssd;
		if(v1 > 0xFFF) v1 = 0xFFF;
		if(v2 > 0xFFF) v2 = 0xFFF;
		uint8_t v1_h = (v1>>8)&0x0F;
		uint8_t v1_l = v1&0xFF;
		uint8_t v2_h = (v2>>8)&0x0F;
		uint8_t v2_l = v2&0xFF;
		data_packet[idx++] = (v1_h<<4) | v2_h;
		data_packet[idx++] = v1_l;
		data_packet[idx++] = v2_l;
	}
	if(cur_send_id == param_skin_res)
	{
		int v = ecg_params.skin_parameter;
		data_packet[idx++] = v>>8;
		data_packet[idx++] = v&0xFF;
		float temp = bmi.T;
		int ti = temp;
		uint8_t t8 = 0;
		if(ti < -20) ti = -20;
		if(ti > 77) ti = 77;
		if(ti < 30) t8 = 20 + ti;
		if(ti > 47) t8 = 220 + (ti-47);
		if(ti >= 30 && ti <= 47)
		{
			t8 = 50 + (temp-30.0)*10.0;
		}
		data_packet[idx++] = t8;
	}
	if(cur_send_id == param_lastRR)
	{
		int v1 = get_RR(1);
		uint8_t v1_h = (v1>>8)&0xFF;
		uint8_t v1_l = v1&0xFF;
		data_packet[idx++] = ecg_params.rr_id;
		data_packet[idx++] = v1_h;
		data_packet[idx++] = v1_l;
	}
	if(cur_send_id == param_imu_acc)
	{
		int ax = 128 + encode_acc(bmi.aX);
		if(ax < 0) ax = 0;
		if(ax > 255) ax = 255;
		int ay = 128 + encode_acc(bmi.aY);
		if(ay < 0) ay = 0;
		if(ay > 255) ay = 255;
		int az = 128 + encode_acc(bmi.aZ);
		if(az < 0) az = 0;
		if(az > 255) az = 255;
		data_packet[idx++] = ax;
		data_packet[idx++] = ay;
		data_packet[idx++] = az;
	}
	if(cur_send_id == param_imu_gyro)
	{
		data_packet[idx++] = encode_16_8(bmi.rwX);
		data_packet[idx++] = encode_16_8(bmi.rwY);
		data_packet[idx++] = encode_16_8(bmi.rwZ);
	}
	if(cur_send_id == param_imu_steps)
	{
		int steps = bmi.step_cnt;
		int g_byte = 0;
		data_packet[idx++] = (steps>>8)&0xFF;
		data_packet[idx++] = steps&0xFF;
		data_packet[idx++] = g_byte;
	}
	if(cur_send_id == param_pnn_bins)
	{
		int bin_v1 = hrv_params.pNN_norm[bin_send_id] * 255.0;
		int bin_v2 = hrv_params.pNN_norm[bin_send_id+1] * 255.0;
		data_packet[idx++] = bin_send_id;
		data_packet[idx++] = bin_v1;
		data_packet[idx++] = bin_v2;
		bin_send_id += 2;
		if(bin_send_id >= 15) bin_send_id = 0;
	}
		
	int pp = ecg_params.ble_buf_pos - send_cnt - 1;
	if(pp < 0) pp += ecg_params.ble_buf_len;
	for(int n = 0; n < send_cnt; n++) //fill ECG data
	{
		data_packet[idx++] = ecg_params.data_buffer_ble[pp]>>8;
		data_packet[idx++] = ecg_params.data_buffer_ble[pp];
		if(++pp >= ecg_params.ble_buf_len) pp = 0;
	}

	//now calculate 2 bytes of checksums - through all and through even bytes
	data_packet[1] = idx+2; //packet length + checksum
	uint8_t check = 0;
	for(int x = 0; x < idx; x++)
		check += data_packet[x];
	data_packet[idx++] = check;
	uint8_t check_e = 0;
	for(int x = 0; x < idx; x+=2)
		check_e += data_packet[x];
	data_packet[idx++] = check_e;
	
	return idx;
}

int data_prepare_packet_r64()
{
	uint8_t idx = 0;
	packet_id++;
	data_packet[idx++] = packet_id;
	data_packet[idx++] = 64; //fill actual value in the end
	data_packet[idx++] = (board_config.unit_id>>24)&0xFF;
	data_packet[idx++] = (board_config.unit_id>>16)&0xFF;
	data_packet[idx++] = (board_config.unit_id>>8)&0xFF;
	data_packet[idx++] = (board_config.unit_id)&0xFF;
	uint8_t send_cnt = 26; //max number of data points that fit in 64 bytes with everything else
	if(dev_config.signal_measurement_mode == signal_measurement_emg)
	{
		data_packet[idx++] = 218; //indicate that it is EMG packet with magic number
		data_packet[idx++] = param_emg_spectrum;
		data_packet[idx++] = (dev_state.battery_mv - 2000)/10;
		data_packet[idx++] = emg_params.data_id;
		data_packet[idx++] = emg_params.level>>8;
		data_packet[idx++] = emg_params.level;
		data_packet[idx++] = emg_params.spectr_scale>>8;
		data_packet[idx++] = emg_params.spectr_scale;
		for(int n = 0; n < 8; n++)
			data_packet[idx++] = emg_params.spectr_buf[n];
		data_packet[1] = idx;
		return idx;	
	}
	//we are sending always the same number of data points, hoping that even if some packets 
	//are lost, they still overlap enough to provide an uninterrupted data stream.
	//with this value, receiver knows what data points are in the packet.
	//50 + points count means that we are using new protocol version
	data_packet[idx++] = 50 + send_cnt;

	data_packet[idx++] = ecg_params.data_id;
	
	//4 bytes are dedicated to various parameters that don't change fast, so we 
	//cycle through them and send values correspondingly
	int cur_send_id;
	param_send_id++;
	if(param_send_id >= param_end) param_send_id = 0;
	cur_send_id = param_send_id;

	data_packet[idx++] = cur_send_id;
	if(cur_send_id == param_batt_bpm)
	{
		data_packet[idx++] = (dev_state.battery_mv-2000)/10;
		data_packet[idx++] = 6; //protocol version_id - it's not the same as device version id
		data_packet[idx++] = ecg_params.BPM_normal;
	}
	if(cur_send_id == param_sdnn)
	{
		int v1 = hrv_params.sdrr;
		int v2 = hrv_params.rmssd;
		if(v1 > 0xFFF) v1 = 0xFFF;
		if(v2 > 0xFFF) v2 = 0xFFF;
		uint8_t v1_h = (v1>>8)&0x0F;
		uint8_t v1_l = v1&0xFF;
		uint8_t v2_h = (v2>>8)&0x0F;
		uint8_t v2_l = v2&0xFF;
		data_packet[idx++] = (v1_h<<4) | v2_h;
		data_packet[idx++] = v1_l;
		data_packet[idx++] = v2_l;
	}
	if(cur_send_id == param_skin_res)
	{
		int v = ecg_params.skin_parameter;
		data_packet[idx++] = v>>8;
		data_packet[idx++] = v&0xFF;
		float temp = bmi.T;
		int ti = temp;
		uint8_t t8 = 0;
		if(ti < -20) ti = -20;
		if(ti > 77) ti = 77;
		if(ti < 30) t8 = 20 + ti;
		if(ti > 47) t8 = 220 + (ti-47);
		if(ti >= 30 && ti <= 47)
		{
			t8 = 50 + (temp-30.0)*10.0;
		}
		data_packet[idx++] = t8;
	}
	if(cur_send_id == param_lastRR)
	{
		int v1 = get_RR(1);
		uint8_t v1_h = (v1>>8)&0xFF;
		uint8_t v1_l = v1&0xFF;
		data_packet[idx++] = ecg_params.rr_id;
		data_packet[idx++] = v1_h;
		data_packet[idx++] = v1_l;
	}
	if(cur_send_id == param_imu_acc)
	{
		int ax = 128 + encode_acc(bmi.aX);
		if(ax < 0) ax = 0;
		if(ax > 255) ax = 255;
		int ay = 128 + encode_acc(bmi.aY);
		if(ay < 0) ay = 0;
		if(ay > 255) ay = 255;
		int az = 128 + encode_acc(bmi.aZ);
		if(az < 0) az = 0;
		if(az > 255) az = 255;
		data_packet[idx++] = ax;
		data_packet[idx++] = ay;
		data_packet[idx++] = az;
	}
	if(cur_send_id == param_imu_gyro)
	{
		data_packet[idx++] = encode_16_8(bmi.rwX);
		data_packet[idx++] = encode_16_8(bmi.rwY);
		data_packet[idx++] = encode_16_8(bmi.rwZ);
	}
	if(cur_send_id == param_imu_steps)
	{
		int steps = bmi.step_cnt;
		int g_byte = 0;
		data_packet[idx++] = (steps>>8)&0xFF;
		data_packet[idx++] = steps&0xFF;
		data_packet[idx++] = g_byte;
	}
	if(cur_send_id == param_pnn_bins)
	{
		int bin_v1 = hrv_params.pNN_norm[bin_send_id] * 255.0;
		int bin_v2 = hrv_params.pNN_norm[bin_send_id+1] * 255.0;
		data_packet[idx++] = bin_send_id;
		data_packet[idx++] = bin_v1;
		data_packet[idx++] = bin_v2;
		bin_send_id += 2;
		if(bin_send_id >= 15) bin_send_id = 0;
	}
		
	int n = 0;
	int pp = ecg_params.buf_pos - send_cnt - 1;
	if(pp < 0) pp += ecg_params.buf_len;
	while(n++ < send_cnt)
	{
		data_packet[idx++] = ecg_params.data_buffer[pp]>>8;
		data_packet[idx++] = ecg_params.data_buffer[pp];
		if(++pp >= ecg_params.buf_len) pp = 0;
	}

	data_packet[1] = idx;
	return idx;
}

//mostly legacy approach
int data_prepare_packet_ble_adv()
{
	//emg mode not supported, so we just ignore if it's turned on
	data_packet[0] = ecg_params.ble_data_id;
	int cur_send_id;
	param_send_id++;
	if(param_send_id >= param_end) param_send_id = 0;
	cur_send_id = param_send_id;

	data_packet[1] = cur_send_id;
	if(cur_send_id == param_batt_bpm)
	{
		data_packet[2] = (dev_state.battery_mv - 2000)/10;
		data_packet[3] = 5; //version_id - this is legacy mode, so version 5 is used
		data_packet[4] = ecg_params.BPM_normal;
	}
	if(cur_send_id == param_sdnn)
	{
		int v1 = hrv_params.sdrr;
		int v2 = hrv_params.rmssd;
		if(v1 > 0xFFF) v1 = 0xFFF;
		if(v2 > 0xFFF) v2 = 0xFFF;
		uint8_t v1_h = (v1>>8)&0x0F;
		uint8_t v1_l = v1&0xFF;
		uint8_t v2_h = (v2>>8)&0x0F;
		uint8_t v2_l = v2&0xFF;
		data_packet[2] = (v1_h<<4) | v2_h;
		data_packet[3] = v1_l;
		data_packet[4] = v2_l;
	}
	if(cur_send_id == param_skin_res)
	{
		int v = ecg_params.skin_parameter;
		data_packet[2] = v>>8;
		data_packet[3] = v&0xFF;
		float temp = bmi.T;
		int ti = temp;
		uint8_t t8 = 0;
		if(ti < -20) ti = -20;
		if(ti > 77) ti = 77;
		if(ti < 30) t8 = 20 + ti;
		if(ti > 47) t8 = 220 + (ti-47);
		if(ti >= 30 && ti <= 47)
		{
			t8 = 50 + (temp-30.0)*10.0;
		}
		data_packet[4] = t8;
	}
	if(cur_send_id == param_lastRR)
	{
		int v1 = get_RR(1);
		int v2 = get_RR(2);
		if(v1 > 0xFFF) v1 = 0xFFF;
		if(v2 > 0xFFF) v2 = 0xFFF;
		uint8_t v1_h = (v1>>8)&0x0F;
		uint8_t v1_l = v1&0xFF;
		uint8_t v2_h = (v2>>8)&0x0F;
		uint8_t v2_l = v2&0xFF;
		data_packet[1] |= ecg_params.rr_id<<4;
		data_packet[2] = (v1_h<<4) | v2_h;
		data_packet[3] = v1_l;
		data_packet[4] = v2_l;
	}
	if(cur_send_id == param_imu_acc)
	{
		int ax = 128 + encode_acc(bmi.aX);
		if(ax < 0) ax = 0;
		if(ax > 255) ax = 255;
		int ay = 128 + encode_acc(bmi.aY);
		if(ay < 0) ay = 0;
		if(ay > 255) ay = 255;
		int az = 128 + encode_acc(bmi.aZ);
		if(az < 0) az = 0;
		if(az > 255) az = 255;
		data_packet[2] = ax;
		data_packet[3] = ay;
		data_packet[4] = az;
	}
	if(cur_send_id == param_imu_steps)
	{
		int steps = bmi.step_cnt;
		int g_byte = 0;
		data_packet[2] = (steps>>8)&0xFF;
		data_packet[3] = steps&0xFF;
		data_packet[4] = g_byte;
	}

	if(cur_send_id == param_pnn_bins)
	{
		int bin_v1 = hrv_params.pNN_norm[bin_send_id] * 255.0;
		int bin_v2 = hrv_params.pNN_norm[bin_send_id+1] * 255.0;
		int bin_v3 = hrv_params.pNN_norm[bin_send_id+2] * 255.0;
		data_packet[1] |= (bin_send_id<<4);
		bin_send_id += 3;
		if(bin_send_id >= 13) bin_send_id = 0;
		data_packet[2] = bin_v1;
		data_packet[3] = bin_v2;
		data_packet[4] = bin_v3;
	}
	//18 bytes available at this point
	//2 will be needed for starting value, 1 for scale parameter
	//15 are left for single-byte data points
	
	int send_seq_cnt = 16;
	int pp = ecg_params.ble_buf_pos - send_seq_cnt - 1;
	if(pp < 0) pp += ecg_params.ble_buf_len;
	uint8_t idx = 5;
	int zero_val = ecg_params.data_buffer_ble[pp];
	data_packet[idx++] = (zero_val >> 8)&0xFF;
	data_packet[idx++] = (zero_val) & 0xFF;
	int start_pp = pp;
	int prev_vv = zero_val;
	int cur_scale = 1;
	int max_dv = 0;
	for(int x = 1; x < send_seq_cnt; x++)
	{
		if(++pp >= ecg_params.ble_buf_len) pp = 0;
		int vv = ecg_params.data_buffer_ble[pp];
		int dv = vv - prev_vv;
		if(dv < 0) dv = -dv;
		if(dv > max_dv) max_dv = dv;
		prev_vv = vv;
	}
	cur_scale = 1 + (max_dv / 127);
	int scale_code = cur_scale;
	if(scale_code > 100) scale_code = 100 + (scale_code-100) / 4;
	data_packet[idx++] = scale_code;
	pp = start_pp;
	prev_vv = zero_val;
	for(int x = 1; x < send_seq_cnt; x++)
	{
		if(++pp >= ecg_params.ble_buf_len) pp = 0;
		int vv = ecg_params.data_buffer_ble[pp];
		int dv = (vv - prev_vv) / cur_scale;
		data_packet[idx++] = dv;
		prev_vv = vv;
	}
	return idx;
}


uint32_t ecg_pack_cnt = 0;
int imu_pack_div = 7;
int hrv_pack_div = 37;

enum
{
	pack_ecg_short = 1,
	pack_imu_rr_short,
	pack_hrv_short,
	pack_ecg_imu,
	pack_ecg_rr,
	pack_ecg_hrv
};

int fill_imu_buf(uint8_t *buf)
{
	int pp = 0;
	buf[pp++] = bmi.raX>>8;
	buf[pp++] = bmi.raX;
	buf[pp++] = bmi.raY>>8;
	buf[pp++] = bmi.raY;
	buf[pp++] = bmi.raZ>>8;
	buf[pp++] = bmi.raZ;
	buf[pp++] = bmi.rwX>>8;
	buf[pp++] = bmi.rwX;
	buf[pp++] = bmi.rwY>>8;
	buf[pp++] = bmi.rwY;
	buf[pp++] = bmi.rwZ>>8;
	buf[pp++] = bmi.rwZ;
	int temp = bmi.T * 10;
	buf[pp++] = temp>>8;
	buf[pp++] = temp;
	int steps = bmi.step_cnt;
	buf[pp++] = steps>>8;
	buf[pp++] = steps;
	return pp;
}
int fill_rr_buf(uint8_t *buf)
{
	int pp = 0;
	buf[pp++] = ecg_params.rr_id;
	uint16_t rr1 = get_RR(1);
	uint16_t rr2 = get_RR(2);
	buf[pp++] = rr1>>8;
	buf[pp++] = rr1;
	buf[pp++] = rr2>>8;
	buf[pp++] = rr2;
	buf[pp++] = ecg_params.BPM_normal;
	buf[pp++] = ecg_params.BPM_momentary;
	buf[pp++] = ecg_params.skin_parameter>>8;
	buf[pp++] = ecg_params.skin_parameter;
	int sdrr = hrv_params.sdrr;
	int rmssd = hrv_params.rmssd;
	buf[pp++] = sdrr>>8;
	buf[pp++] = sdrr;
	buf[pp++] = rmssd>>8;
	buf[pp++] = rmssd;
	buf[pp++] = r_detector.R_time>>8;
	buf[pp++] = r_detector.R_time;
	buf[pp++] = (dev_state.battery_mv - 2000)/10;
	return pp;
}
int fill_imu_rr_buf(uint8_t *buf)
{
	int pp = 0;
	uint16_t aX = (bmi.raX+32768)>>4;
	uint16_t aY = (bmi.raY+32768)>>4;
	uint16_t aZ = (bmi.raZ+32768)>>4;
	uint16_t wX = (bmi.rwX+32768)>>4;
	uint16_t wY = (bmi.rwY+32768)>>4;
	uint16_t wZ = (bmi.rwZ+32768)>>4;
	buf[pp++] = aX>>4;//8 higher bits
	buf[pp++] = (aX<<4) | (aY>>8); //4+4 bits
	buf[pp++] = aY; //8 lower bits
	buf[pp++] = aZ>>4;
	buf[pp++] = (aZ<<4) | (wX>>8);
	buf[pp++] = wX;
	buf[pp++] = wY>>4;
	buf[pp++] = (wY<<4) | (wZ>>8);
	buf[pp++] = wZ;
	//9 bytes used	
	int temp = bmi.T * 10 - 200; //we are interested in body temperature, so +20...+45 C is enough
	if(temp < 0) temp = 0;
	if(temp > 255) temp = 255;
	buf[pp++] = temp;
	int steps = bmi.step_cnt;
	buf[pp++] = steps>>8;
	buf[pp++] = steps;
	//12 bytes used	
	buf[pp++] = ecg_params.rr_id;
	uint16_t rr1 = get_RR(1);
	uint16_t rr2 = get_RR(2);
	if(rr1 > 0xFFF) rr1 = 0xFFF;
	if(rr2 > 0xFFF) rr2 = 0xFFF;
	buf[pp++] = rr1>>4;
	buf[pp++] = (rr1<<4) | (rr2>>8);
	buf[pp++] = rr2;
	//16 bytes used
	buf[pp++] = ecg_params.BPM_normal;
	buf[pp++] = ecg_params.skin_parameter>>8;
	buf[pp++] = ecg_params.skin_parameter;
	//19 bytes used	
	return pp;
}
int fill_hrv_buf(uint8_t *buf)
{
	int pp = 0;
	for(int n = 0; n < 15; n++)
		buf[pp++] = hrv_params.pNN_norm[n];
	//15 bytes used
	int sdrr = hrv_params.sdrr;
	int rmssd = hrv_params.rmssd;
	if(sdrr > 0xFFF) sdrr = 0xFFF; //both should be <4095 for correct signal
	if(rmssd > 0xFFF) rmssd = 0xFFF;
	buf[pp++] = sdrr>>4;
	buf[pp++] = (sdrr<<4) | (rmssd>>8);
	buf[pp++] = rmssd;
	//18 bytes used
//	int rdet = r_detector.R_time;
//	if(rdet > 255) rdet = 255;
//	buf[pp++] = rdet; //no room for detector info ((
	buf[pp++] = (dev_state.battery_mv - 2000)/10;
	return pp;
}

int data_prepare_ble_conn()
{
	int our_mtu = ble_get_current_MTU();
//	if(our_mtu < 40)
	if(1) //for now, use only short MTUs even if device supports more
	{
		ecg_pack_cnt++;
		data_ecg_ch[0] = pack_ecg_short;
		if(ecg_pack_cnt%imu_pack_div == 0)
		{
			data_ecg_ch[0] = pack_imu_rr_short;
			fill_imu_rr_buf(data_ecg_ch+1);
		}
		else if(ecg_pack_cnt%hrv_pack_div == 0)
		{
			data_ecg_ch[0] = pack_hrv_short;
			fill_hrv_buf(data_ecg_ch+1);
		}
		else
		{
			int ecg_points = 13;//20 - 1 type - 2 data id - 2 first point - 1 bpm - 1 scale = 13 data points

			int pos = ecg_params.ble_buf_pos - ecg_points - 1;
			if(pos < 0) pos += ecg_params.ble_buf_len;
			data_ecg_ch[0] = pack_ecg_short;
			int pp = 1;
			data_ecg_ch[pp++] = ecg_params.ble_data_id>>8;
			data_ecg_ch[pp++] = ecg_params.ble_data_id;
			int v_start = ecg_params.data_buffer_ble[pos];

			int cur_scale = 1;
			int max_dv = 0;
			int min_dv = 0;
			int prev_v = v_start;
			pos = ecg_params.ble_buf_pos - ecg_points;
			if(pos < 0) pos += ecg_params.ble_buf_len;
			int start_pos = pos;
			for(int n = 0; n < ecg_points; n++)
			{
				int v = ecg_params.data_buffer_ble[pos];
				int dv = v - prev_v;
				prev_v = v;
				if(dv > max_dv) max_dv = dv;
				if(dv < min_dv) min_dv = dv;
				if(++pos >= ecg_params.ble_buf_len) pos = 0;
			}
			int sp = 1 + (max_dv / 127);
			int sn = 1 - (min_dv / 127);
			if(sp > sn) cur_scale = sp;
			else cur_scale = sn;
			int scale_code = cur_scale;
			if(scale_code > 100) scale_code = 100 + (scale_code-100) / 4;
			data_ecg_ch[pp++] = scale_code;

			data_ecg_ch[pp++] = v_start>>8;
			data_ecg_ch[pp++] = v_start;

			prev_v = v_start;
			pos = start_pos;
			for(int n = 0; n < ecg_points; n++)
			{
				int v = ecg_params.data_buffer_ble[pos];
				int dv = v - prev_v;
				prev_v = v;
				data_ecg_ch[pp++] = 128 + (dv / cur_scale);
				if(++pos >= ecg_params.ble_buf_len) pos = 0;
			}
			data_ecg_ch[pp++] = ecg_params.BPM_momentary;
		}
		return 20;
	}
	else
	{
		int header_sz = 2;
		int add_sz = 16;
		int ecg_sz = our_mtu - add_sz - header_sz;
		int pack_type = pack_ecg_imu;
		ecg_pack_cnt++;
		if(ecg_pack_cnt%7 == 0) pack_type = pack_ecg_rr;
		if(ecg_pack_cnt%19 == 0) pack_type = pack_ecg_hrv;
		data_ecg_ch[0] = pack_type;
		data_ecg_ch[1] = ecg_sz;
		int pp = 2;
		int ecg_points = (ecg_sz-2)/2;
		int pos = ecg_params.ble_buf_pos - ecg_points - 1;
		if(pos < 0) pos += ecg_params.ble_buf_len;
		data_ecg_ch[pp++] = ecg_params.ble_data_id>>8;
		data_ecg_ch[pp++] = ecg_params.ble_data_id;
		for(int n = 0; n < ecg_points; n++)
		{
			data_ecg_ch[pp++] = ecg_params.data_buffer_ble[pos]>>8;
			data_ecg_ch[pp++] = ecg_params.data_buffer_ble[pos];
			if(++pos >= ecg_params.ble_buf_len) pos = 0;
		}
		if(pack_type == pack_ecg_imu)
			pp += fill_imu_buf(data_ecg_ch+pp);
		if(pack_type == pack_ecg_rr)
			pp += fill_rr_buf(data_ecg_ch+pp);
		if(pack_type == pack_ecg_hrv)
		{
			data_ecg_ch[pp++] = hrv_params.pNN_bins;
			for(int n = 0; n < hrv_params.pNN_bins; n++)
				data_ecg_ch[pp++] = hrv_params.pNN_norm[n];
		}
		return pp;
	}
}

uint8_t *data_get_packet_r32()
{
	return data_packet;
}
uint8_t *data_get_packet_r64()
{
	return data_packet;
}
uint8_t *data_get_packet_ble_adv() 
{
	return data_packet;
}
uint8_t *data_get_ble_ecg_ch()
{
	return data_ecg_ch;
}
uint8_t *data_get_ble_rr_ch()
{
	return data_rr_ch;
}
uint8_t *data_get_ble_hrv_ch()
{
	return data_hrv_ch;
}
uint8_t *data_get_ble_imu_ch()
{
	return data_imu_ch;
}


