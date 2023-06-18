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
	Serial.printf("this is the on board temp %f C\n",get_onboard_temperature_c());
	Serial.printf("this is the temp #1 %f C\n",get_thermistor_temperature_c(1));
	Serial.printf("this is the temp #2 %f C\n",get_thermistor_temperature_c(2));
	Serial.printf("this is the DHT temp %f C\n",get_dht22_temperature_c());
	Serial.printf("this is the DHT humiditity #2 %lf \n",get_dht22_humidity_rh_pct());
	Serial.printf("this is the bmp280 pressure %lf Pa\n",get_bmp280_pressure_pa());
	Serial.printf("this is the bmp280 temp %lf C\n",get_bmp280_temperature_c());
	Serial.printf("this is the bmp280 altitude %lf m\n",get_bmp280_altitude());
	Serial.printf("this is the battery %lf mV\n",get_battery_voltage());
	Serial.printf("this is the RTF %d \n",is_switch_ready_to_fly());
	// Serial.printf("this is ADC beign tested %d \n",analogReadMilliVolts(27));
	set_debug_led_state(1);
	delay(250);
	set_debug_led_state(0);
	delay(3000);

	lora_do_test_and_log();

	lora_send_str_and_seq("HELP_ADC");

	Serial.println("Done loop().");
}
