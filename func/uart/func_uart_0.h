#ifndef FUNC_UART_H
#define FUNC_UART_H

#include "main.h"
#include "circle_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_NUM 2
#define UART_SIZE 255

typedef struct {
    circle_buffer_t rxbuffer;
    circle_buffer_t txbuffer;
    UART_HandleTypeDef *huart;

    circle_buffer_t txbuffer_num;
    circle_buffer_t rxbuffer_num;

} func_uart_base_t;


bool Func_Uart_Init(func_uart_base_t *dev, uint8_t *rxbuffer, uint8_t rx_length,
                    uint8_t *txbuffer, uint8_t tx_length,
                    UART_HandleTypeDef *huart);
bool Func_Uart_Transmit(func_uart_base_t *dev, const uint8_t *frame, uint8_t length);
bool Func_Uart_Receive(func_uart_base_t *dev, uint8_t *frame, uint8_t length);
 
#ifdef __cplusplus
}
#endif

#endif /* FUNC_UART_H */
