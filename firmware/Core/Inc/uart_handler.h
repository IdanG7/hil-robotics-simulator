#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 128

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile bool tx_busy;
} UART_Handle_t;

bool UART_Init(UART_Handle_t *handle, UART_HandleTypeDef *huart);
bool UART_BytesAvailable(UART_Handle_t *handle);

/**
 * @brief Read a single byte from the UART receive buffer.
 * @note PRECONDITION: Caller MUST check UART_BytesAvailable() returns true
 *       before calling this function. Reading from an empty buffer results
 *       in undefined behavior.
 */
uint8_t UART_ReadByte(UART_Handle_t *handle);
bool UART_SendPacket(UART_Handle_t *handle, const uint8_t *data, uint16_t len);
void UART_RxCallback(UART_Handle_t *handle);
void UART_TxCallback(UART_Handle_t *handle);

#endif // UART_HANDLER_H
