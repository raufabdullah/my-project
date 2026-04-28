#ifndef APP_UART_H
#define APP_UART_H

/**
 * @file app_uart.h
 * @brief UART communication interfaces.
 */

/**
 * @brief Prints a message over the primary UART interface.
 * @param msg Null-terminated string to send.
 */
void UART_Print(const char *msg);

#endif