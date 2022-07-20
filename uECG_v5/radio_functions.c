#include <stdlib.h> //NULL
#include <stdio.h> //sprintf
#include "nrf.h"

#include "radio_functions.h"

#include "urf_timer.h"
#include "urf_radio.h"
#include "urf_ble_peripheral.h"
#include "urf_star_protocol.h"
#include "ble_const.h"
#include "urf_ble_att_process.h"

#include "board_config.h"
#include "data_packing.h"
#include "ecg_processor.h"
#include "bmi160.h"

int cur_radio_mode = radio_mode_off;

sService GAP_service;
sService HeartRate_service; //0x180D 
sService Battery_service; //0x180F
sService uECG_service;

sCharacteristic gap_name_ch; //0x2A00
sCharacteristic gap_appearance_ch; //0x2A01
sCharacteristic heart_rate_ch; //0x2A37
sCharacteristic battery_lvl_ch; //0x2A19

sCharacteristic ecg_data_ch;
sCharacteristic rr_data_ch;
sCharacteristic hrv_data_ch;
sCharacteristic imu_data_ch;
sCharacteristic uecg_settings_ch;

uint8_t ble_mac[6];

void radio_set_mode(int mode)
{
	cur_radio_mode = mode;
	if(mode == radio_mode_off)
	{
		rf_disable();
		return;
	}
	if(mode == radio_mode_direct32)
	{
		rf_disable();
		rf_override_irq(NULL);
		schedule_event_stop();
		NRF_RADIO->POWER = 0;
		delay_ms(2);
		rf_init(21, 250, 0);
//		rf_init_ext(21, 250, 0, 0, 0, 0, 255);
		rf_listen();
	}
	if(mode == radio_mode_direct64)
	{
		rf_disable();
		rf_override_irq(NULL);
		schedule_event_stop();
		NRF_RADIO->POWER = 0;
		delay_ms(2);
		star_init(21, 1000, 2000, 0);
		star_set_id(board_config.unit_id);
	}
	if(mode == radio_mode_ble_adv)
	{
		rf_disable();
		rf_override_irq(NULL);
		schedule_event_stop();
		NRF_RADIO->POWER = 0;
		delay_ms(2);
		ble_init_radio();
		uint32_t id = board_config.unit_id;
/*		ble_mac[0] = id;
		ble_mac[1] = id>>8;
		ble_mac[2] = id>>16;
		ble_mac[3] = id>>24;
		ble_mac[4] = 0xC6;
		ble_mac[5] = 0x0E | 0b11000000; //non-resolvable static*/
		ble_set_our_mac(ble_mac);
	}
}

uint32_t radio_last_send_time = 0;
uint32_t ble_ecg_ch_time = 0;
uint32_t ble_hrv_ch_time = 0;
uint32_t ble_rr_ch_time = 0;
uint32_t ble_imu_ch_time = 0;

//this function takes care of timing depending on selected mode
//so the main cycle just needs to call it often enough
//and it also takes data according to current mode from
//data_packing utils so main cycle doesn't need to pass it
//any arguments
void radio_process_sending()
{
	if(cur_radio_mode == radio_mode_off) return; //nothing to do if mode is not set
	uint32_t ms = millis();
	//arduino-compatible, sending data periodically without any requests from the other side
	if(cur_radio_mode == radio_mode_direct32)
	{
		if(ms - radio_last_send_time < 5) return;
		radio_last_send_time = ms;
		int len = data_prepare_packet_r32();
		uint8_t *packet = data_get_packet_r32();
		rf_send(packet, len);
		return;
	}
	//direct radio using star protocol - base station sends requests to all units it's aware of
	//and when request is received, it will be responded by the protocol. Some packets can be
	//never sent if star_queue_send is called before we got request from the base - which should
	//be ok, data packing has a lot of redundancy
	if(cur_radio_mode == radio_mode_direct64)
	{
		star_loop_step();
		if(ms - radio_last_send_time < 5) return;
		radio_last_send_time = ms;
		int len = data_prepare_packet_r64();
		uint8_t *packet = data_get_packet_r64();
		star_queue_send(packet, len);
	}
	//in 
	if(cur_radio_mode == radio_mode_ble_adv)
	{
		if(ble_get_conn_state())
			cur_radio_mode = radio_mode_ble_conn;

		if(ms - radio_last_send_time < 15) return;
		radio_last_send_time = ms;

		uint8_t pdu[40];
		uint8_t payload[40];
		
		for(int x = 0; x < 6; x++)
			payload[x] = ble_mac[x];
		int pp = 6;
		payload[pp++] = 0x02;
		payload[pp++] = 0x01;
		payload[pp++] = 0b0110; //general discovery, br/edr not supported

		static int send_name = 0;
		send_name++;
		if(!dev_config.ecg_advertising_enabled) send_name = 100; //simple way 
		//to always send device name only, without ever sending ecg data via advertising
		if(send_name > 10)
		{
			send_name = 0;
			uint8_t name[32];
			int nlen = sprintf(name, "uECG v4.%d", board_config.version);
			payload[pp++] = nlen+1;
			payload[pp++] = 0x08;
			for(int x = 0; x < nlen; x++)
				payload[pp++] = name[x];
			payload[pp++] = 3;
			payload[pp++] = 0x03; //complete list of 16-bit UUIDs
			payload[pp++] = 0x0D; //heart rate UUID
			payload[pp++] = 0x18;
			payload[pp++] = 0;
		}
		else
		{
			int rem_len = 37-2-6-3-1;
			payload[pp++] = rem_len;
			payload[pp++] = 0xFF;
			rem_len--;
			int len = data_prepare_packet_ble_adv();
			uint8_t *packet = data_get_packet_ble_adv();
			for(int x = 0; x < len; x++)
			{
				if(x >= rem_len) break;
				payload[pp++] = packet[x];
			}
		}
		static int adv_ch = 37; //cycle through advertising channels
		int len = ble_prepare_adv_pdu(pdu, 35, payload, BLE_ADV_IND_TYPE, 0, 1);
		ble_LL_send_PDU(0x8E89BED6, len, pdu, adv_ch);
		adv_ch++;
		if(adv_ch > 39) adv_ch = 37;
	}
	if(cur_radio_mode == radio_mode_ble_conn)
	{
		if(!ble_get_conn_state())
			cur_radio_mode = radio_mode_ble_adv;
		if(ms - radio_last_send_time < 5) return;
		int pack_size = data_prepare_ble_conn();
		radio_last_send_time = ms;
		
		if(heart_rate_ch.had_read)
		{
			ecg_params.unsent_RR_cnt = 0;
			heart_rate_ch.had_read = 0;
		}
		if(ms - ble_hrv_ch_time > 713)
		{
			ble_hrv_ch_time = ms;
			uint8_t *ch = data_get_ble_hrv_ch();
			if(hrv_data_ch.descriptor_values[0] > 0)
			{
				for(int x = 0; x < 20; x++)
				{
					if(hrv_data_ch.value[x] != ch[x])
						hrv_data_ch.changed = 1; //it's updated with each R peak - so possibly it wasn't updated yet
					hrv_data_ch.value[x] = ch[x];				
				}
			}
			heart_rate_ch.value[1] = ecg_params.BPM_normal;
			heart_rate_ch.val_length = 2;
			if(ecg_params.unsent_RR_cnt > 0)
			{
				int rr_fill = ecg_params.unsent_RR_cnt; 
				if(rr_fill > 5) rr_fill = 5; //max number of RR intervals to send
				for(int r = 0; r < rr_fill; r++)
				{
					uint16_t RR = get_RR(rr_fill - r); //older first
					heart_rate_ch.value[2 + r*2] = RR; //lso first
					heart_rate_ch.value[2 + r*2 + 1] = RR>>8;
					heart_rate_ch.val_length += 2;
				}
			}
			if(heart_rate_ch.descriptor_values[0] > 0)
				heart_rate_ch.changed = 1;
			int batt_mv = dev_state.battery_mv;
			int batt_perc;
			if(batt_mv > 4000)
				batt_perc = 95 + (batt_mv - 4000) / 30;
			else if(batt_mv > 3500)
				batt_perc = 10 + (batt_mv - 3500) / 5.9;
			else batt_perc = (batt_mv - 3200) / 30;
			if(batt_perc < 0) batt_perc = 0;
			if(batt_perc > 100) batt_perc = 100;
			battery_lvl_ch.value[0] = batt_perc;
			
			if(uecg_settings_ch.had_write)
			{
				uecg_settings_ch.had_write = 0;
				//implementation pending - need to deserialize value and apply settings
			}
		}
		if(ms - ble_ecg_ch_time > 10)
		{
			ble_ecg_ch_time = ms;
		
			uint8_t *ch = data_get_ble_ecg_ch();
			for(int x = 0; x < pack_size; x++)
				ecg_data_ch.value[x] = ch[x];
			if(ecg_data_ch.descriptor_values[0] > 0)
			{
				ecg_data_ch.changed = 1;
			}
		}
	}
}

void radio_fill_ble_services()
{
	uint8_t er_key[34];
	uint8_t ir_key[34];
	sprintf(er_key, "1C3DB1A287B3102BEA89AA49A3990989");
	sprintf(ir_key, "A1EF042887B3102BEA89AA49A3990989");
	ble_peripheral_set_ER(er_key);
	ble_peripheral_set_IR(ir_key);

	ble_peripheral_generate_mac(ble_mac);
	ble_set_our_mac(ble_mac);
	
	GAP_service.handle = 0x01;
	GAP_service.type = 0x2800;
	GAP_service.group_end = 0x0F;
	GAP_service.uuid_16 = 0x1800;
	ble_add_service(&GAP_service);

	HeartRate_service.handle = 0x10;
	HeartRate_service.type = 0x2800;
	HeartRate_service.group_end = 0x1F;
	HeartRate_service.uuid_16 = 0x180D;
	ble_add_service(&HeartRate_service);

	Battery_service.handle = 0x20;
	Battery_service.type = 0x2800;
	Battery_service.group_end = 0x2F;
	Battery_service.uuid_16 = 0x180F;
	ble_add_service(&Battery_service);
	
	uECG_service.handle = 0x30;
	uECG_service.type = 0x2800;
	uECG_service.group_end = 0xFFFF;
	uECG_service.uuid_16 = 0;
	//93375900 F2298B49 B39744B5 899B8600
	ble_uuid_from_text(uECG_service.uuid_128, "93375900-F229-8B49-B397-44B5899B8600");
	ble_add_service(&uECG_service);
	
	gap_name_ch.handle = 0x02;
	gap_name_ch.value_handle = 0x03;
	gap_name_ch.uuid_16 = 0x2A00;
	gap_name_ch.descriptor_count = 0;
	gap_name_ch.val_length = sprintf(gap_name_ch.value, "uECG v4.%d", board_config.version);
	gap_name_ch.val_type = VALUE_TYPE_UTF8;
	gap_name_ch.properties = CHARACTERISTIC_READ;
	ble_add_characteristic(&gap_name_ch);
	GAP_service.char_idx[0] = gap_name_ch.mem_idx;
	
	gap_appearance_ch.handle = 0x04;
	gap_appearance_ch.value_handle = 0x05;
	gap_appearance_ch.uuid_16 = 0x2A01;
	gap_appearance_ch.descriptor_count = 0;
	gap_appearance_ch.val_length = 2;
	gap_appearance_ch.val_type = VALUE_TYPE_UINT16;
	gap_appearance_ch.value[0] = 0x40; //generic hear rate sensor
	gap_appearance_ch.value[1] = 0x03;
	gap_appearance_ch.properties = CHARACTERISTIC_READ;
	ble_add_characteristic(&gap_appearance_ch);
	GAP_service.char_idx[1] = gap_appearance_ch.mem_idx;
	GAP_service.char_count = 2;
	

	heart_rate_ch.handle = 0x11;
	heart_rate_ch.value_handle = 0x12;
	heart_rate_ch.descriptor_count = 1;
	heart_rate_ch.descriptor_uuids[0] = 0x2902;
	heart_rate_ch.descriptor_handles[0] = 0x13;
	heart_rate_ch.descriptor_values[0] = 0;
	heart_rate_ch.uuid_16 = 0x2A37;
	heart_rate_ch.val_length = 2;
	heart_rate_ch.val_type = VALUE_TYPE_UTF8;
	heart_rate_ch.value[0] = 0b10000;//flags: 8-bit BPM, RR intervals present, contact detection not supported
	heart_rate_ch.value[1] = 80; //default BPM value - will be updated
	heart_rate_ch.properties = CHARACTERISTIC_NOTIFY;
	ble_add_characteristic(&heart_rate_ch);
	HeartRate_service.char_idx[0] = heart_rate_ch.mem_idx;
	HeartRate_service.char_count = 1;

	battery_lvl_ch.handle = 0x21;
	battery_lvl_ch.value_handle = 0x22;
	battery_lvl_ch.descriptor_count = 0;
	battery_lvl_ch.uuid_16 = 0x2A19;
	battery_lvl_ch.val_length = 1;
	battery_lvl_ch.val_type = VALUE_TYPE_UTF8;
	battery_lvl_ch.value[0] = 50; //by default fill with 50% - will be updated
	battery_lvl_ch.properties = CHARACTERISTIC_READ;
	ble_add_characteristic(&battery_lvl_ch);
	Battery_service.char_idx[0] = battery_lvl_ch.mem_idx;
	Battery_service.char_count = 1;
	
	uECG_service.char_count = 0;
	ecg_data_ch.handle = 0x31 + (3*uECG_service.char_count);
	ecg_data_ch.value_handle = ecg_data_ch.handle + 1;
	ecg_data_ch.descriptor_count = 1;
	ecg_data_ch.descriptor_uuids[0] = 0x2902;
	ecg_data_ch.descriptor_handles[0] = ecg_data_ch.handle+2;
	ecg_data_ch.descriptor_values[0] = 0;
	ecg_data_ch.uuid_16 = 0;
	ble_uuid_from_text(ecg_data_ch.uuid_128, "FC7A850D-C1A5-F61F-0DA7-9995621FBD00");
	ecg_data_ch.val_type = VALUE_TYPE_UTF8;
	ecg_data_ch.val_length = 20;
	ecg_data_ch.properties = CHARACTERISTIC_READ | CHARACTERISTIC_NOTIFY;
	ble_add_characteristic(&ecg_data_ch);
	uECG_service.char_idx[uECG_service.char_count] = ecg_data_ch.mem_idx;
	uECG_service.char_count++;
/*
	rr_data_ch.handle = 0x31 + (3*uECG_service.char_count);
	rr_data_ch.value_handle = rr_data_ch.handle + 1;
	rr_data_ch.descriptor_count = 1;
	rr_data_ch.descriptor_uuids[0] = 0x2902;
	rr_data_ch.descriptor_handles[0] = rr_data_ch.handle+2;
	rr_data_ch.descriptor_values[0] = 0;
	rr_data_ch.uuid_16 = 0;
	ble_uuid_from_text(rr_data_ch.uuid_128, "261827CA-070B-5038-4A12-BB5A33EBA700");
	rr_data_ch.val_type = VALUE_TYPE_UTF8;
	rr_data_ch.val_length = 20;
	rr_data_ch.properties = CHARACTERISTIC_READ | CHARACTERISTIC_NOTIFY;
	ble_add_characteristic(&rr_data_ch);
	uECG_service.char_idx[uECG_service.char_count] = rr_data_ch.mem_idx;
	uECG_service.char_count++;

	hrv_data_ch.handle = 0x31 + (3*uECG_service.char_count);
	hrv_data_ch.value_handle = hrv_data_ch.handle + 1;
	hrv_data_ch.descriptor_count = 1;
	hrv_data_ch.descriptor_uuids[0] = 0x2902;
	hrv_data_ch.descriptor_handles[0] = hrv_data_ch.handle+2;
	hrv_data_ch.descriptor_values[0] = 0;
	hrv_data_ch.uuid_16 = 0;
	ble_uuid_from_text(hrv_data_ch.uuid_128, "D3DA8951-F54F-40C3-CF21-846AA3769700");
	hrv_data_ch.val_type = VALUE_TYPE_UTF8;
	hrv_data_ch.val_length = 20;
	hrv_data_ch.properties = CHARACTERISTIC_READ | CHARACTERISTIC_NOTIFY;
	ble_add_characteristic(&hrv_data_ch);
	uECG_service.char_idx[uECG_service.char_count] = hrv_data_ch.mem_idx;
	uECG_service.char_count++;

	imu_data_ch.handle = 0x31 + (3*uECG_service.char_count);
	imu_data_ch.value_handle = imu_data_ch.handle + 1;
	imu_data_ch.descriptor_count = 1;
	imu_data_ch.descriptor_uuids[0] = 0x2902;
	imu_data_ch.descriptor_handles[0] = imu_data_ch.handle+2;
	imu_data_ch.descriptor_values[0] = 0;
	imu_data_ch.uuid_16 = 0;
	ble_uuid_from_text(imu_data_ch.uuid_128, "F77BA62E-097A-B892-F9E4-7C1EDA5A4900");
	imu_data_ch.val_type = VALUE_TYPE_UTF8;
	imu_data_ch.val_length = 20;
	imu_data_ch.properties = CHARACTERISTIC_READ | CHARACTERISTIC_NOTIFY;
	ble_add_characteristic(&imu_data_ch);
	uECG_service.char_idx[uECG_service.char_count] = imu_data_ch.mem_idx;
	uECG_service.char_count++;*/

	uecg_settings_ch.handle = 0x31 + (3*uECG_service.char_count);
	uecg_settings_ch.value_handle = uecg_settings_ch.handle + 1;
	uecg_settings_ch.descriptor_count = 0;
	uecg_settings_ch.uuid_16 = 0;
	ble_uuid_from_text(uecg_settings_ch.uuid_128, "4AC2927D-0469-BD24-286C-ECFBA5CA9000");
	uecg_settings_ch.val_type = VALUE_TYPE_UTF8;
	uecg_settings_ch.val_length = 20;
	uecg_settings_ch.properties = CHARACTERISTIC_READ | CHARACTERISTIC_WRITE;
	ble_add_characteristic(&uecg_settings_ch);
	uECG_service.char_idx[uECG_service.char_count] = uecg_settings_ch.mem_idx;
	uECG_service.char_count++;
}