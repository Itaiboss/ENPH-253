/**
 * @file logs.h
 * @brief contains macro for console and display logs. 
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#define LOG_BUFF_LEN 400
#define CONSOLE_LOG(tag, formatted_text,...)  sendLog(tag, formatted_text, ##__VA_ARGS__)
void sendLog(const char* log_tag, const char* formatted_text,...);
