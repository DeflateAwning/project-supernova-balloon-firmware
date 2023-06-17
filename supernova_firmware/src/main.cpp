#include <Arduino.h>
#include "hardware_drivers.h"

// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	Serial.println("Start setup().");

	// init the LEDs
	do_pin_init_actions();

	Serial.println("Done setup().");
}

// the loop function runs over and over again forever
void loop() {
	Serial.println("Start loop().");
	
	set_debug_led_state(1);
	delay(250);
	set_debug_led_state(0);
	delay(1000);

	Serial.println("Done loop().");
}
