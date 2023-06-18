
#pragma once // only allow include once
#include "Arduino.h"
#include <thermistor.h>
#include "DHT.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Hardware drivers for the Project Supernova Project

#define PIN_TEMP_V_PLUS 25
#define PIN_ONBOARD_TEMPERATURE_SENSOR 34
#define PIN_TEMP_SENSE_1 33
#define PIN_TEMP_SENSE_2 32
#define PIN_BATT_SENSE 27
#define PIN_DHT_DATA 23 // pin was moved from D12 to D23
#define PIN_RTF_SWITCH 18
#define PIN_DEBUG_LED 5
#define PIN_GPS_UART_MOSI 2 // software serial
#define PIN_GPS_UART_MISO 4
#define PIN_HEATER_ENABLE 14

#define LORA_UART_CHANNEL 2 // UART channel number 2

#define TEMP_SENSOR_5 80 // This is thermistor 0
#define TEMP_SENSOR_6 80 // This is thermistor 0
#define DHTPIN 12     // Digital pin connected to the DHT senso
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// inits the core hardware pins
void do_pin_init_actions();

// reads and returns the onboard temperature of U4 (LM335D)
double get_onboard_temperature_c();

// reads and returns the temperature of the two thermistor probes
double get_thermistor_temperature_c(int sensor_number);

// reads and returns the DHT22 temperature
double get_dht22_temperature_c();

// reads and returns the DHT22 humidity (from 0 to 100)
double get_dht22_humidity_rh_pct();

// reads and returns the BMP280 temperature
double get_bmp280_temperature_c();

// reads and returns the BMP280 pressure (in Pascals)
double get_bmp280_pressure_pa();

// reads and returns the BMP280 altitude (in mwtwea)
double get_bmp280_altitude();

// reads and returns the battery voltage
double get_battery_voltage();

// reads and returns the state of the "ready to fly" switch
bool is_switch_ready_to_fly();

// TODO setup GPS module
// TODO setup LoRa module


// sets the board heater state to either on or off
void set_board_heater_state(bool turn_on);

// sets the D5 debug LED state
void set_debug_led_state(bool turn_on);
