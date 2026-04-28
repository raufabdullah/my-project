#include "main.h"
#include "app_uart.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

/**
 * @brief Prints a null-terminated string over UART2 via blocking transmission.
 * @param msg The string to be transmitted.
 */
void UART_Print(const char *msg) {
    // Transmit string characters over UART, blocking until complete.
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}