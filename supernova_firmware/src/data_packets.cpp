#include "data_packets.h"

void data_packet_1_to_hex_string(data_packet_1_t* packet, char* hex_str_buffer) {
	uint8_t size_of_packet = sizeof(struct data_packet_1_t);

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
