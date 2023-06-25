#include "data_packets.h"

void data_packet_1_to_hex_string(struct data_packet_1_t packet, char* hex_str_buffer) {
	uint8_t size_of_packet = sizeof(packet);

	// Source: https://stackoverflow.com/a/43242170
	// convert to byte array
	char* buffer = new char[size_of_packet];
    memset(buffer, 0x00, size_of_packet);
	memcpy(buffer, &packet, size_of_packet);

	// convert byte array to hex string
	memset(hex_str_buffer, 0x00, MAX_DATA_PACKET_HEX_STRING_LENGTH-1); // zero the buffer
	for (uint8_t i = 0; i < size_of_packet; i++) {
		sprintf(hex_str_buffer + (i * 2), "%02X", buffer[i]);
	}

	delete[] buffer; // free memory

}

void packet_debug_log_sizes() {
	Serial.println("Start packet_debug_log_sizes()");

	struct data_packet_1_t dp1;

	// log size of the data
	Serial.printf("sizeof(data_packet_1) = %d bytes\n", sizeof(dp1));

	Serial.println("Finished packet_debug_log_sizes()");
	
}

void log_data_packet_1_to_serial(struct data_packet_1_t packet) {

	Serial.println("Start log_data_packet_1_to_serial()");

	Serial.printf("PACKET 1: packet_type = %d\n", packet.packet_type);
	Serial.printf("PACKET 1: millis_since_boot = %d\n", packet.millis_since_boot);
	Serial.printf("PACKET 1: packet_seq_num = %d\n", packet.packet_seq_num);
	Serial.printf("PACKET 1: lora_dr_value = %d\n", packet.lora_dr_value);
	Serial.printf("PACKET 1: themistor_1_temperature_c = %d\n", packet.themistor_1_temperature_c);
	Serial.printf("PACKET 1: themistor_2_temperature_c = %d\n", packet.themistor_2_temperature_c);
	Serial.printf("PACKET 1: dht_temperature_c = %d\n", packet.dht_temperature_c);
	Serial.printf("PACKET 1: dht_humidity_pct = %d\n", packet.dht_humidity_pct);
	Serial.printf("PACKET 1: bmp_pressure_pa = %f\n", packet.bmp_pressure_pa);
	Serial.printf("PACKET 1: bmp_temperature_c = %f\n", packet.bmp_temperature_c);
	Serial.printf("PACKET 1: battery_voltage_mv = %d\n", packet.battery_voltage_mv);
	Serial.printf("PACKET 1: mcu_internal_temperature_c = %d\n", packet.mcu_internal_temperature_c);
	Serial.printf("PACKET 1: is_switch_rtf = %d\n", packet.is_switch_rtf);
	
	Serial.printf("PACKET 1: gps_latitude_degrees_x1e6 = %d\n", packet.gps_latitude_degrees_x1e6);
	Serial.printf("PACKET 1: gps_longitude_degrees_x1e6 = %d\n", packet.gps_longitude_degrees_x1e6);
	Serial.printf("PACKET 1: gps_altitude_cm = %d\n", packet.gps_altitude_cm);
	Serial.printf("PACKET 1: gps_speed_100ths_of_knot = %d\n", packet.gps_speed_100ths_of_knot);
	Serial.printf("PACKET 1: gps_course_100ths_of_degree = %d\n", packet.gps_course_100ths_of_degree);
	Serial.printf("PACKET 1: gps_satellite_count = %d\n", packet.gps_satellite_count);
	Serial.printf("PACKET 1: gps_hdop_cm = %d\n", packet.gps_hdop_cm);
	Serial.printf("PACKET 1: gps_fix_date_epoch_time_sec = %lu\n", packet.gps_fix_date_epoch_time_sec);
	Serial.printf("PACKET 1: gps_fix_date_age_ms = %d\n", packet.gps_fix_date_age_ms);

}
