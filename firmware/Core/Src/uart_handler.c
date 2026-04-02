#include "uart_handler.h"
#include <string.h>

bool UART_Init(UART_Handle_t *handle, UART_HandleTypeDef *huart) {
    handle->huart = huart;
    handle->rx_head = 0;
    handle->rx_tail = 0;
    handle->tx_busy = false;
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, handle->rx_buffer, UART_RX_BUFFER_SIZE);
    return (status == HAL_OK);
}

bool UART_BytesAvailable(UART_Handle_t *handle) {
    handle->rx_head = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(handle->huart->hdmarx);
    return (handle->rx_head != handle->rx_tail);
}

uint8_t UART_ReadByte(UART_Handle_t *handle) {
    uint8_t byte = handle->rx_buffer[handle->rx_tail];
    handle->rx_tail = (handle->rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return byte;
}

bool UART_SendPacket(UART_Handle_t *handle, const uint8_t *data, uint16_t len) {
    if (len > UART_TX_BUFFER_SIZE) {
        return false;
    }

    // Use blocking transmit - simpler and more reliable for now
    HAL_StatusTypeDef status = HAL_UART_Transmit(handle->huart, (uint8_t*)data, len, 100);
    return (status == HAL_OK);
}

void UART_RxCallback(UART_Handle_t *handle) {
    // Called from HAL_UART_RxCpltCallback - nothing needed for circular DMA
}

void UART_TxCallback(UART_Handle_t *handle) {
    handle->tx_busy = false;
}
