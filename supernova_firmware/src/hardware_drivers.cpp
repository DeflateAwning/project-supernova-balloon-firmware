#include "hardware_drivers.h"
#include "Arduino.h"


DHT dht(PIN_DHT_DATA, DHT_TYPE);
Adafruit_BMP280 bmp;

HardwareSerial gps_serial(1); // use UART1 (default pins not exposed, but that's actually fine)

TinyGPSPlus gps;

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
	gps_serial.begin(gps_baud_rate, SERIAL_8N2, PIN_GPS_UART_MISO, PIN_GPS_UART_MOSI, false);

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

// reads bytes from GPS serial line, and passes them to the gps parser
void gps_update_data() {
	uint8_t incoming_byte;
	bool received_data = false;
	
	Serial.print("RAW GPS DATA: ");

	while (gps_serial.available() > 0) {
		incoming_byte = gps_serial.read();
		gps.encode(incoming_byte);
		Serial.write(incoming_byte);

		received_data = true;
	}

	if (received_data) {
		Serial.println("\nEND RAW GPS DATA");
	}
	else {
		Serial.println();
	}
}

// log all GPS info
void gps_log_all_info() {

	// Latitude in degrees (double)
	Serial.print("GPS DATA: Latitude= "); 
	Serial.print(gps.location.lat(), 6);      
	// Longitude in degrees (double)
	Serial.print(" Longitude= "); 
	Serial.println(gps.location.lng(), 6); 

	// Raw latitude in whole degrees
	// Serial.print("GPS DATA: Raw latitude = "); 
	// Serial.print(gps.location.rawLat().negative ? "-" : "+");
	// Serial.println(gps.location.rawLat().deg); 
	// // ... and billionths (u16/u32)
	// Serial.println(gps.location.rawLat().billionths);

	// Raw longitude in whole degrees
	Serial.print("GPS DATA: Raw longitude = "); 
	Serial.print(gps.location.rawLng().negative ? "-" : "+");
	Serial.println(gps.location.rawLng().deg); 
	// ... and billionths (u16/u32)
	Serial.println(gps.location.rawLng().billionths);

	// Raw date in DDMMYY format (u32)
	Serial.print("GPS DATA: Raw date DDMMYY = ");
	Serial.println(gps.date.value()); 

	// Year (2000+) (u16)
	Serial.print("GPS DATA: Year = "); 
	Serial.println(gps.date.year()); 
	// Month (1-12) (u8)
	Serial.print("GPS DATA: Month = "); 
	Serial.println(gps.date.month()); 
	// Day (1-31) (u8)
	Serial.print("GPS DATA: Day = "); 
	Serial.println(gps.date.day()); 

	// Raw time in HHMMSSCC format (u32)
	Serial.print("GPS DATA: Raw time in HHMMSSCC = "); 
	Serial.println(gps.time.value()); 

	// Hour (0-23) (u8)
	Serial.print("GPS DATA: Hour = "); 
	Serial.println(gps.time.hour()); 
	// Minute (0-59) (u8)
	Serial.print("GPS DATA: Minute = "); 
	Serial.println(gps.time.minute()); 
	// Second (0-59) (u8)
	Serial.print("GPS DATA: Second = "); 
	Serial.println(gps.time.second()); 
	// 100ths of a second (0-99) (u8)
	Serial.print("GPS DATA: Centisecond = "); 
	Serial.println(gps.time.centisecond()); 

	// Raw speed in 100ths of a knot (i32)
	Serial.print("GPS DATA: Raw speed in 100ths/knot = ");
	Serial.println(gps.speed.value()); 
	// Speed in knots (double)
	Serial.print("GPS DATA: Speed in knots/h = ");
	Serial.println(gps.speed.knots()); 
	// Speed in miles per hour (double)
	Serial.print("GPS DATA: Speed in miles/h = ");
	Serial.println(gps.speed.mph()); 
	// Speed in meters per second (double)
	Serial.print("GPS DATA: Speed in m/s = ");
	Serial.println(gps.speed.mps()); 
	// Speed in kilometers per hour (double)
	Serial.print("GPS DATA: Speed in km/h = "); 
	Serial.println(gps.speed.kmph()); 

	// Raw course in 100ths of a degree (i32)
	Serial.print("GPS DATA: Raw course in degrees = "); 
	Serial.println(gps.course.value()); 
	// Course in degrees (double)
	Serial.print("GPS DATA: Course in degrees = "); 
	Serial.println(gps.course.deg()); 

	// Raw altitude in centimeters (i32)
	Serial.print("GPS DATA: Raw altitude in centimeters = "); 
	Serial.println(gps.altitude.value()); 
	// Altitude in meters (double)
	Serial.print("GPS DATA: Altitude in meters = "); 
	Serial.println(gps.altitude.meters()); 
	// Altitude in miles (double)
	Serial.print("GPS DATA: Altitude in miles = "); 
	Serial.println(gps.altitude.miles()); 
	// Altitude in kilometers (double)
	Serial.print("GPS DATA: Altitude in kilometers = "); 
	Serial.println(gps.altitude.kilometers()); 
	// Altitude in feet (double)
	Serial.print("GPS DATA: Altitude in feet = "); 
	Serial.println(gps.altitude.feet()); 

	// Number of satellites in use (u32)
	Serial.print("GPS DATA: Number of satellites in use = "); 
	Serial.println(gps.satellites.value()); 

	// Horizontal Dim. of Precision (100ths-i32)
	Serial.print("GPS DATA: HDOP (100ths of a meter) = "); 
	Serial.println(gps.hdop.value());

}

bool gps_is_location_updated() {
	return gps.location.isUpdated();
}

