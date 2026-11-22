// #include "func_uart_0.h"
// #include <string.h>

// // 内部注册表：用于在中断回调中反查实例
// static func_uart_base_t *s_uart_map[UART_NUM] = {NULL};
// static uint8_t s_txbuffer_num[UART_NUM][UART_SIZE]; 
// static uint8_t s_rxbuffer_num[UART_NUM][UART_SIZE];

// /**
//  * @brief 初始化串口实例并注册
//  */
// bool Func_Uart_Init(func_uart_base_t *dev, uint8_t *rxbuffer, uint8_t rx_length,
//                     uint8_t *txbuffer, uint8_t tx_length,
//                     UART_HandleTypeDef *huart)
// {
//     // 1. 防御性检查
//     if (dev == NULL || rxbuffer == NULL || txbuffer == NULL || huart == NULL) return false;
//     if (rx_length == 0 || tx_length == 0) return false;

//     // 2. 检查此 huart 是否已被其他 dev 注册（防止冲突）
//     for (int i = 0; i < UART_NUM; i++) {
//         if (s_uart_map[i] != NULL && s_uart_map[i]->huart == huart) {
//             return false; 
//         }
//     }

//     // 3. 绑定用户传入的缓冲区和句柄
//     dev->huart = huart;
//     dev->rxbuffer = Circle_Buffer_Init(rxbuffer, rx_length);
//     dev->txbuffer = Circle_Buffer_Init(txbuffer, tx_length);

//     // 4. 注册到静态数组中（找空位存放 dev 指针）
//     bool registered = false;
//     for (int i = 0; i < UART_NUM; i++) {
//         if (s_uart_map[i] == NULL) {
//             s_uart_map[i] = dev;
//             registered = true;

//             dev->txbuffer_num = Circle_Buffer_Init(s_txbuffer_num[i],UART_SIZE);
//             dev->rxbuffer_num = Circle_Buffer_Init(s_rxbuffer_num[i],UART_SIZE);

//             break;
//         }
//     }
//     if (!registered) return false; // 槽位已满

//     // 5. 启动底层的 DMA 空闲接收
//     if (HAL_UARTEx_ReceiveToIdle_DMA(huart, dev->rxbuffer.address, rx_length) != HAL_OK) {
//         return false;
//     }
    
//     // 6. 关闭半满中断（防止重复触发，降低 CPU 开销）
//     __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

//     return true;
// }

// /**
//  * @brief 发送数据
//  */
// bool Func_Uart_Transmit(func_uart_base_t *dev, const uint8_t *frame, uint8_t length)
// {
//     if (dev == NULL || frame == NULL || length == 0) return false;
    
//     if(Circle_Buffer_IsFull(&dev->txbuffer_num)){
//         return false;
//     }

//     if(Circle_Buffer_Free(&dev->txbuffer) < length){
//         return false;
//     }

//     Circle_Buffer_Put_Buffer(&dev->txbuffer, frame, length);
//     Circle_Buffer_Put(&dev->txbuffer_num,length);
    
//     return true;
// }

// /**
//  * @brief 接收数据
//  */
// bool Func_Uart_Receive(func_uart_base_t *dev, uint8_t *frame, uint8_t length)
// {
//     if (dev == NULL || frame == NULL || length == 0) return false;

//     uint8_t date;
//     if(Circle_Buffer_Peek(&dev->rxbuffer_num,0,&date))
//     {
//         if(date > length){
//             return false;
//         }
//     }

//     date=Circle_Buffer_Get(&dev->rxbuffer_num,0);
//     Circle_Buffer_Get_Buffer(&dev->rxbuffer, frame, length);
    
//     return true;
// }
