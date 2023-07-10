/**
 * @file logs.h
 * @brief contains macro for console and display logs. 
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#include <Wire.h>
#include <logs.h>
#pragma once
static const char* LOG_TAG = "LOGS";
/**
 * @brief Sends a log message by Serial
 * 
 * @param log_tag level of log to be used (1 to 5 which are ERROR, WARNING, INFORMATION, DEBUG, VERBOSE)
 * @param formatted_text text to be sent as a log. It can be plain text or formatted text with %s, %d, etc
*/
void sendLog(const char* log_tag, const char* formatted_text,...) {
  char log_buffer[LOG_BUFF_LEN];
  va_list arg;
  va_start(arg, formatted_text);
  vsnprintf(log_buffer, LOG_BUFF_LEN, formatted_text, arg);
  va_end(arg);
  Serial.print(log_buffer);
}