#include "hardware_drivers.h"
#include "Arduino.h"

thermistor therm1(PIN_TEMP_SENSE_1, 11);
thermistor therm2(PIN_TEMP_SENSE_2, 80);

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp;

void do_pin_init_actions() {
	Serial.println("Start do_pin_init_actions()");

	pinMode(PIN_TEMP_V_PLUS, OUTPUT);
	pinMode(PIN_DEBUG_LED, OUTPUT);
	
	pinMode(PIN_ONBOARD_TEMPERATURE_SENSOR, INPUT);
	pinMode(PIN_TEMP_SENSE_1, INPUT);
	pinMode(PIN_TEMP_SENSE_2, INPUT);
	pinMode(PIN_BATT_SENSE, INPUT);
	pinMode(PIN_DHT_DATA, INPUT);
	pinMode(PIN_RTF_SWITCH, INPUT);
	pinMode(PIN_ONBOARD_TEMPERATURE_SENSOR, INPUT);
	
	pinMode(PIN_GPS_UART_MOSI, OUTPUT);
	pinMode(PIN_GPS_UART_MISO, INPUT);

	

	digitalWrite(PIN_TEMP_V_PLUS, 1); //move this to read function

	 bmp.begin(0x76);
	 bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
	Serial.println("Done do_pin_init_actions()");
}





void set_debug_led_state(bool turn_on) {
	// absurdly simple
	digitalWrite(PIN_DEBUG_LED, turn_on);
}

double get_onboard_temperature_c(){
	return (double)analogReadMilliVolts(PIN_ONBOARD_TEMPERATURE_SENSOR)/3.3;
}

// Needs fixing
double get_thermistor_temperature_c(int sensor_number){
	if (sensor_number == 1){
		return ((double)therm1.analog2temp());
	}
	else
		return (double)therm2.analog2temp();
}

double get_dht22_temperature_c(){
	return (double)dht.readTemperature();
}
double get_dht22_humidity_rh_pct(){
	return (double)dht.readHumidity();
}

double get_bmp280_temperature_c(){
	return (double) bmp.readTemperature();
}

double get_bmp280_pressure_pa(){
	return (double) bmp.readPressure();
}

double get_bmp280_altitude(){
	return (double) bmp.readAltitude();
}

double get_battery_voltage(){
	return ((double) analogReadMilliVolts(PIN_BATT_SENSE))/0.3197;
}

bool is_switch_ready_to_fly(){
	return digitalRead(PIN_RTF_SWITCH);
}

void set_board_heater_state(bool turn_on){
	digitalWrite(PIN_HEATER_ENABLE, turn_on);
}

