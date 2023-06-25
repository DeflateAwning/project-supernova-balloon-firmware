#pragma once
#include <Arduino.h>

#define MAX_DATA_PACKET_HEX_STRING_LENGTH 255

// floats are 6 decimal places? doubles are 15?
struct data_packet_1_t
{
	uint8_t packet_type; // always 1 for this struct
	uint32_t millis_since_boot;
	uint16_t packet_seq_num;
	uint8_t lora_dr_value;

	int8_t themistor_1_temperature_c;
	int8_t themistor_2_temperature_c;
	int8_t dht_temperature_c;
	int8_t dht_humidity_pct;
	float_t bmp_pressure_pa;
	float_t bmp_temperature_c;
	uint16_t battery_voltage_mv;
	int8_t mcu_internal_temperature_c;
	uint8_t is_switch_rtf; // TODO pack other bools in here, if we want

	// HERE is a good split if we ever wanted 2 different types of packets
	int32_t gps_latitude_degrees_x1e6;
	int32_t gps_longitude_degrees_x1e6;
	int32_t gps_altitude_cm;
	int32_t gps_speed_100ths_of_knot;
	int32_t gps_course_100ths_of_degree;

	uint16_t gps_satellite_count;
	int32_t gps_hdop_cm;

	// uint8_t gps_date_month;
	// uint8_t gps_date_day;
	// uint8_t gps_date_hour;
	// uint8_t gps_date_minute;
	// uint8_t gps_date_second;
	// uint8_t gps_fix_date_centisecond; // always 0 for this GPS, I guess
	
	uint32_t gps_fix_date_epoch_time_sec;
	uint32_t gps_fix_date_age_ms;

};

// converts a data_packet_1_t to a byte array, and then to a null-terminated hex string
// provide a buffer at least MAX_DATA_PACKET_HEX_STRING_LENGTH bytes long
void data_packet_1_to_hex_string(struct data_packet_1_t packet, char* hex_str_buffer);

// logs out to serial the contents of the packet
void log_data_packet_1_to_serial(struct data_packet_1_t);

// log the sizes of each packet, just for info
void packet_debug_log_sizes();
