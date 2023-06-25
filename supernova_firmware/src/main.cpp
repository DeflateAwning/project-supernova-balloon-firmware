#include <Arduino.h>
#include "hardware_drivers.h"
#include "lora_comms.h"

// TODO add hourly reboot, just in case

uint16_t data_packet_1_seq_number = 0; // wrap-around is fine
uint8_t lora_dr_value = 2;

// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	Serial.println("Start setup().");

	// init the pins
	do_pin_init_actions();
	init_lora_serial();

	packet_debug_log_sizes();

	delay(2000); // wait for LoRa boot
	lora_exec_command_and_receive_response("AT+RESET", 5000); // reboot the module

	// do lora setup
	lora_do_test_and_log();
	lora_set_private_config();
	lora_exec_command_and_receive_response("AT+ID", 1000); // receive back the device IDs
	lora_set_network_config();
	lora_join();

	Serial.println("Done setup().");
}

// the loop function runs over and over again forever
void loop() {
	Serial.println("Start loop().");

	set_debug_led_state(1);
	delay(250);
	set_debug_led_state(0);
	delay(400);

	bool is_lora_serial_available = false;
	while (Serial.available()) {
		if (!is_lora_serial_available) {
			Serial.println("Running user's LoRa command.");
		}
		
		is_lora_serial_available = true;
		// pass onto LoRa
		char command_buffer[MAX_LORA_RESPONSE_LENGTH];
		receive_uart_data(Serial, command_buffer);
		lora_exec_command_and_receive_response(command_buffer, 5000);
	}
	if (is_lora_serial_available) {
		Serial.println("Done running user's LoRa command.");
	}

	lora_join();
	lora_do_test_and_log();

	//lora_send_str_and_seq("HELP_ADC");

	log_each_sensor_value();

	
	gps_update_data();
	// if (gps_is_location_updated()) gps_log_all_info();

	// show how long the payload is allowed to be
	lora_exec_command_and_receive_response("AT+LW=LEN", 500);

	// generate and send the packet
	struct data_packet_1_t packet_1 = make_data_packet_1(data_packet_1_seq_number++, lora_dr_value);
	char packet_1_hex_str[MAX_DATA_PACKET_HEX_STRING_LENGTH];
	data_packet_1_to_hex_string(packet_1, packet_1_hex_str);
	log_data_packet_1_to_serial(packet_1);
	lora_exec_tx_command_and_receive_response("AT+MSGHEX", packet_1_hex_str);

	delay(1000);

	Serial.println("Done loop().");
}
