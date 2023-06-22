#pragma once
#include "Arduino.h"

// Command Reference: https://files.seeedstudio.com/products/317990687/res/LoRa-E5%20AT%20Command%20Specification_V1.0%20.pdf

#define MAX_LORA_RESPONSE_LENGTH 600 // actually 528 characters, per Command Reference, Section 2.1

void init_lora_serial();
void receive_uart_data(HardwareSerial SerialPort, char* dest_array);
bool lora_do_test_and_log();
void lora_set_private_config();
void lora_set_network_config();
void lora_join();

// Sends to the current LoRaWAN network the str_to_send arg, concatenated with a uint sequence number (starting at 0 from boot)
void lora_send_str_and_seq(const char* str_to_send);

void lora_exec_command_and_receive_response(const char* command_str, uint16_t delay_ms);
void lora_exec_command_and_receive_response(const char* command_str, char* result_dest, uint16_t delay_ms);
