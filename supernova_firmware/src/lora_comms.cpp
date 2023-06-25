
#include "Arduino.h"
#include <stdio.h>

#include "lora_comms.h"
#include "private_lorawan_config.h"

// LoRa Communication for Supernova

#define LoraSerial Serial2 // define the LoRa Serial port ID


uint16_t text_message_seq_num = 0;

void init_lora_serial() {
	Serial.println("Start init_lora_serial()");
	LoraSerial.begin(9600);
	Serial.println("Done init_lora_serial()");

}

void lora_prep_to_send_command() {
	// clear any pending actions
	//LoraSerial.println("");
	LoraSerial.flush();
	delay(100);

	// check for any incoming content
	char buffer[MAX_LORA_RESPONSE_LENGTH];
	receive_uart_data(LoraSerial, buffer);
	if (buffer[0] != 0) {
		Serial.print("lora_prep_to_send_command(): content was found in the buffer:|");
		Serial.print(buffer);
		Serial.println("|");
	}
}

// read data from SerialPort into a cstr, while stripping out leading and trailing newlines
void receive_uart_data(HardwareSerial SerialPort, char* dest_array) {
	uint16_t index = 0;
	char c = 0;
	while (SerialPort.available()) {
		c = SerialPort.read();
		
		if (c == '\r') {
			continue;
		}

		if (c == '\n' && index == 0) {
			// Ignore first \n.
			continue;
		}

		dest_array[index++] = c;
	}
	dest_array[index] = 0; // write cstr null terminator

	if ((index > 0) && (c == '\n')) {
		// remove trailing newline
		dest_array[index-1] = 0;
	}
}

void lora_exec_tx_command_and_receive_response(const char* at_command_str, const char* message_payload_str) {
	uint16_t delay_ms = 5000; // TODO confirm this is a good value
	
	// prep command
	char command_str[MAX_LORA_RESPONSE_LENGTH];
	sprintf(command_str, "%s=\"%s\"", at_command_str, message_payload_str); // form string like: AT+MSG="Hello World"
	
	// send command and receive response
	char rx_buffer[MAX_LORA_RESPONSE_LENGTH];
	lora_exec_command_and_receive_response(command_str, rx_buffer, delay_ms);

	// TODO deal with responses, like "Length error N", N>0
	
	
	// TODO deal with responses, like "Length error N", N=0 (action: send an "AT+MSG" dummy command to clear uplink buffer)
}

void lora_exec_command_and_receive_response(const char* command_str, uint16_t delay_ms) {
	char rx_buffer[MAX_LORA_RESPONSE_LENGTH];
	return lora_exec_command_and_receive_response(command_str, rx_buffer, delay_ms);
}

void lora_exec_command_and_receive_response(const char* command_str, char* result_dest, uint16_t delay_ms) {
	lora_prep_to_send_command();
	
	// send the command
	Serial.print("MCU->LoRa: >>");
	Serial.println(command_str);
	LoraSerial.println(command_str); // println executes it
	LoraSerial.flush();

	delay(delay_ms); // TODO shorten this delay, looking for "+JOIN: Joined already" or ": Done"
	
	// receive the response
	receive_uart_data(LoraSerial, result_dest);
	Serial.print("LoRa->MCU: >>");
	Serial.print(result_dest);
	Serial.println("<|");
}

// sends the "AT" command, and expects the "+AT: OK" response; returns True if success, or False is failure
bool lora_do_test_and_log() {
	// send the test command
	char buffer[MAX_LORA_RESPONSE_LENGTH];
	lora_exec_command_and_receive_response("AT", buffer, 250);

	// TODO add return check
	return true;

}

void lora_set_private_config() {
	LoraSerial.println("Start lora_set_private_config()");

	char command_to_send[255];

	// send DevEui
	sprintf(command_to_send, "AT+ID=DevEui,\"%s\"", lora_private_dev_eui);
	lora_exec_command_and_receive_response(command_to_send, 1000);
	
	// send AppEui
	sprintf(command_to_send, "AT+ID=AppEui,\"%s\"", lora_private_app_eui);
	lora_exec_command_and_receive_response(command_to_send, 1000);
	
	// send AppKey
	sprintf(command_to_send, "AT+KEY=APPKEY,\"%s\"", lora_private_app_key);
	lora_exec_command_and_receive_response(command_to_send, 1000);

	LoraSerial.println("Done lora_set_private_config()");
}


void lora_set_network_config() {
	LoraSerial.println("Start lora_set_network_config()");

	lora_exec_command_and_receive_response("AT+DR=US915", 500);
	lora_set_dr(2); // TODO remove this hard-code, and instead cycle through them
	// lora_exec_command_and_receive_response("AT+DR=SF10", 500); // lower spreading factor = lower transmission range (but it's set through the DRx command)
	lora_exec_command_and_receive_response("AT+CH=NUM,8-15", 500);
	lora_exec_command_and_receive_response("AT+MODE=LWOTAA", 500);

	// set TX Power in dBm
	lora_exec_command_and_receive_response("AT+POWER=22", 500); // appears that 20 dBm is the default and 22 dBm is the max

	// print out a few confirmation commands
	lora_exec_command_and_receive_response("AT+LW=LEN", 500); // view max length of packet
	lora_exec_command_and_receive_response("AT+DR", 500); // view data rate settings

	LoraSerial.println("Done lora_set_network_config()");

}

void lora_set_dr(uint8_t dr) {
	char command_to_send[MAX_LORA_RESPONSE_LENGTH];
	sprintf(command_to_send, "AT+DR=DR%d", dr);
	lora_exec_command_and_receive_response(command_to_send, 500);
}

void lora_join() {
	lora_exec_command_and_receive_response("AT+JOIN", 15000);

	// TODO handle response
	// TODO return faster than delay if it says "already joined"
}

void lora_send_str_and_seq(const char* str_to_send) {
	// NOTE: thing function is mostly for testing purposes, and is now deprecated
	char command_to_send[255];
	sprintf(command_to_send, "AT+MSG=\"%s%d\"", str_to_send, text_message_seq_num++);
	lora_exec_command_and_receive_response(command_to_send, 5000);

	// TODO make this use the new lora_exec_tx_command_and_receive_response() function

	// TODO handle "Please join network first" response
	// TODO figure out what's wrong with "Length error"

}

