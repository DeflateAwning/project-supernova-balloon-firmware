#include "hardware_drivers.h"
#include "Arduino.h"


DHT dht(PIN_DHT_DATA, DHT_TYPE);
Adafruit_BMP280 bmp;


void do_pin_init_actions() {
	Serial.println("Start do_pin_init_actions()");

	pinMode(PIN_TEMP_V_PLUS, OUTPUT);
	pinMode(PIN_DEBUG_LED, OUTPUT);
	
	pinMode(PIN_TEMP_SENSE_1, INPUT);
	pinMode(PIN_TEMP_SENSE_2, INPUT);
	pinMode(PIN_BATT_SENSE, INPUT);
	// pinMode(PIN_DHT_DATA, INPUT);
	pinMode(PIN_RTF_SWITCH, INPUT);
	// pinMode(PIN_ONBOARD_TEMPERATURE_SENSOR, INPUT);
	
	pinMode(PIN_GPS_UART_MOSI, OUTPUT);
	pinMode(PIN_GPS_UART_MISO, INPUT);

	// set onboard ADC settings
	analogReadResolution(ADC_BIT_COUNT);
	analogSetCycles(32); // default is 8
	// analogSetAttenuation(...); // default is 11dB

	digitalWrite(PIN_TEMP_V_PLUS, 1); // TODO move this to read function maybe

	 bmp.begin(0x76);
	 bmp.setSampling(
		Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
		Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
		Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
		Adafruit_BMP280::FILTER_X16,      /* Filtering. */
		Adafruit_BMP280::STANDBY_MS_500
	); /* Standby time. */

	dht.begin();

	Serial.println("Done do_pin_init_actions()");
}



void set_debug_led_state(bool turn_on) {
	// absurdly simple
	digitalWrite(PIN_DEBUG_LED, turn_on);
}

uint16_t analog_read_mv(uint8_t pin_number) {
	//return (analogRead(pin_number) * 3300) / (1 << ADC_BIT_COUNT);

	// source: https://deepbluembedded.com/esp32-adc-tutorial-read-analog-voltage-arduino/
	uint16_t analog_read_val = analogRead(pin_number);

	adc_unit_t adc_unit = ADC_UNIT_1;
	if (pin_number == 2 || pin_number == 4 || pin_number == 25 || pin_number == 26 || pin_number == 27 || (pin_number >= 12 && pin_number <= 15)) {
		adc_unit = ADC_UNIT_2;
	}

	esp_adc_cal_characteristics_t adc_chars;
	esp_adc_cal_characterize(adc_unit, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
	return (esp_adc_cal_raw_to_voltage(analog_read_val, &adc_chars));
}

double convert_analog_read_mv_to_temperature_c_stein(uint16_t analog_read_mv) {
	// Source: https://arduino.stackexchange.com/a/37136

	double reading;
	double stein;

	// reverse-apply voltage divider logic
	reading = (3300.0 / ((double)analog_read_mv)) - 1.0;
	reading = themistor_res_fixed / reading;

	// apply the S-H equation
	stein = reading / themistor_res_therm;    // (R/Ro)
	stein = log(stein);             // ln(R/Ro)
	stein /= themistor_therm_b;               // 1/B * ln(R/Ro)
	stein += 1.0 / (25.0 + 273.15); // + (1/To)
	stein = 1.0 / stein;            // Invert
	stein -= 273.15;                // convert to C

	return stein;
}

double get_thermistor_temperature_c(int sensor_number) {	
	uint16_t analog_read_val_mv = 0;
	
	if (sensor_number == 1) {
		analog_read_val_mv = analog_read_mv(PIN_TEMP_SENSE_1);
	}
	else if (sensor_number == 2) {
		analog_read_val_mv = analog_read_mv(PIN_TEMP_SENSE_2);
	}
	else {
		return -1;
	}

	return convert_analog_read_mv_to_temperature_c_stein(analog_read_val_mv);
}

double get_dht22_temperature_c() {
	return dht.readTemperature();
}
double get_dht22_humidity_rh_pct() {
	return dht.readHumidity();
}

double get_bmp280_temperature_c() {
	return (double) bmp.readTemperature();
}

double get_bmp280_pressure_pa() {
	return (double) bmp.readPressure();
}

double get_bmp280_altitude() {
	return (double) bmp.readAltitude();
}

double get_battery_voltage() {
	// Divider Ratio: 4.7k/22.7k = 0.207048x
	// 12V battery/supply turns into 2.48V max
	return ((double) analog_read_mv(PIN_BATT_SENSE))*(22.7/4.7);
}

bool is_switch_ready_to_fly() {
	return digitalRead(PIN_RTF_SWITCH);
}

void set_board_heater_state(bool turn_on){
	digitalWrite(PIN_HEATER_ENABLE, turn_on);
}

float get_internal_temperature_c() {
	float temp_c = temperatureRead();

	if (temp_c > 53.3 && temp_c < 53.4) {
		// special case where the internal temp sensor isn't configured on this board or something
		return -99;
	}
	return temp_c;

}

void do_sensor_test() {
	
	Serial.printf("SENSOR TEST: probe 1 and 2 voltage: %d mV, %d mV\n", analog_read_mv(PIN_TEMP_SENSE_1), analog_read_mv(PIN_TEMP_SENSE_2));
	Serial.printf("SENSOR TEST: probe 1 and 2 temperatures: %f C,  %f C\n", get_thermistor_temperature_c(1), get_thermistor_temperature_c(2));
	Serial.printf("SENSOR TEST: DHT temp %f C\n", get_dht22_temperature_c());
	Serial.printf("SENSOR TEST: DHT humiditity %lf \n", get_dht22_humidity_rh_pct());
	Serial.printf("SENSOR TEST: bmp280 pressure %lf Pa\n",get_bmp280_pressure_pa());
	Serial.printf("SENSOR TEST: bmp280 temp %lf C\n",get_bmp280_temperature_c());
	Serial.printf("SENSOR TEST: bmp280 altitude %lf m\n",get_bmp280_altitude());
	Serial.printf("SENSOR TEST: battery voltage: %lf mV; raw BATT_SENSE read: %d mV\n", get_battery_voltage(), analog_read_mv(PIN_BATT_SENSE));
	Serial.printf("SENSOR TEST: RTF switch state: %d \n", is_switch_ready_to_fly());
	Serial.printf("SENSOR TEST: ESP32 internal temp: %lf C\n", get_internal_temperature_c());
	
}
