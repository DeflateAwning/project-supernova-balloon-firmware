#include <Arduino.h>
#include "hardware_drivers.h"
#include "lora_comms.h"

// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	Serial.println("Start setup().");

	// init the LEDs
	do_pin_init_actions();
	init_lora_serial();

	delay(2000); // wait for LoRa boot
	lora_do_test_and_log();
	lora_set_private_config();

	// receive back the device IDs
	lora_exec_command_and_receive_response("AT+ID", 1000);

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

	delay(1000);

	Serial.println("Done loop().");
}
