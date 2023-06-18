#pragma once
#include "Arduino.h"

void init_lora_serial();
void receive_uart_data(HardwareSerial SerialPort, char* dest_array);
bool lora_do_test_and_log();
void lora_set_private_config();
void lora_set_network_config();
void lora_join();
void lora_send_str_and_seq(const char* str_to_send);

void lora_exec_command_and_receive_response(const char* command_str, uint16_t delay_ms);
void lora_exec_command_and_receive_response(const char* command_str, char* result_dest, uint16_t delay_ms);
