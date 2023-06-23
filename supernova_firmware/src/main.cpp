#include <Arduino.h>
#include "hardware_drivers.h"
#include "lora_comms.h"

// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	Serial.println("Start setup().");

	// init the 
	do_pin_init_actions();
	init_lora_serial();

	packet_debug_log_sizes();

	delay(2000); // wait for LoRa boot
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

	lora_join();
	lora_do_test_and_log();

	lora_send_str_and_seq("HELP_ADC");

	do_sensor_test();

	gps_update_data();
	if (gps_is_location_updated()) {
		gps_log_all_info();
		// TODO store to the upcoming data frame, and store how long it's been since it's been updated
	}
	else {
		Serial.println("GPS DATA: No update.");
	}

	delay(1000);

	Serial.println("Done loop().");
}
