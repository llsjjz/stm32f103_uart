#ifndef FUNC_UART_0_H
#define FUNC_UART_0_H

#include "main.h"
#include "circle_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_NUM 2
#define UART_MAX 255
#define UART_MAXNUM 255

typedef struct {
    circle_buffer_t rxbuffer;
    circle_buffer_t txbuffer;
    UART_HandleTypeDef *huart;

    circle_buffer_t txbuffer_num;
    circle_buffer_t rxbuffer_num;

    /* ---- 内部状态（由模块维护，请勿手动修改） ---- */
    uint8_t slot;               /* 静态中转数组的槽位索引 */
    uint32_t rx_drop_cnt;       /* 接收帧因空间不足被整帧丢弃的次数 */
    bool tx_busy;               /* true 表示 DMA 正在发送一帧 */
    uint16_t tx_frame_len;      /* 当前 DMA 正在发送的帧长，发送完成前不释放 */
    uint32_t err_cnt;           /* HAL 启动/错误回调累计次数 */
    bool initialized;           /* 实例是否已成功初始化 */

} func_uart_base_t;


bool Func_Uart_Init(func_uart_base_t *dev, uint8_t *rxbuffer, uint16_t rx_length,
                    uint8_t *txbuffer, uint16_t tx_length,
                    UART_HandleTypeDef *huart);
void Func_Uart_DeInit(func_uart_base_t *dev);
bool Func_Uart_Transmit(func_uart_base_t *dev, const uint8_t *frame, uint16_t length);
bool Func_Uart_Receive(func_uart_base_t *dev, uint8_t *frame, uint16_t length, uint16_t *out_length);
 
#ifdef __cplusplus
}
#endif

#endif /* FUNC_UART_0_H */
