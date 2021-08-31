#include <stdint.h>

int data_prepare_packet_r32();
int data_prepare_packet_r64();
int data_prepare_packet_ble_adv();
int data_prepare_ble_conn();

uint8_t *data_get_packet_r32();
uint8_t *data_get_packet_r64();
uint8_t *data_get_packet_ble_adv();

uint8_t *data_get_ble_ecg_ch();
uint8_t *data_get_ble_rr_ch();
uint8_t *data_get_ble_hrv_ch();
uint8_t *data_get_ble_imu_ch();
