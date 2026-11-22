/*
 * ============================ func_uart_0 原理 ============================
 *
 * 一、帧格式（本模块的约定）
 *   - 不依赖包头/包尾，也不在总线上附加长度前缀；
 *   - 接收端以 IDLE / DMA 传输完成（TC）作为帧边界：
 *     本轮 DMA 收到的整段数据 = 一帧；
 *   - 因此发送方不应在一帧中间出现明显停顿（超过 1 个字符时间），
 *     否则会被拆成两帧；单帧长度不得超过 UART_MAX（255）。
 *
 * 二、接收数据流
 *   UART RX -> DMA -> s_rxbuffer[slot]（静态中转）
 *     -> HAL_UARTEx_RxEventCallback：IDLE/TC 时把 [0, Size) 作为一帧
 *     -> rxbuffer（数据环形队列）+ rxbuffer_num（2 字节小端长度队列）
 *     -> Func_Uart_Receive() 一次取出一帧
 *   HT 事件不是帧边界：直接忽略，不提交也不重启。
 *   两个队列任一空间不足：整帧丢弃并累加 rx_drop_cnt，绝不写入半帧。
 *
 * 三、发送数据流
 *   用户 -> Func_Uart_Transmit()
 *     -> 校验长度与空间后整帧写入 txbuffer + txbuffer_num
 *     -> 若发送空闲：窥视帧长，拷入 s_txbuffer[slot]
 *     -> HAL_UART_Transmit_DMA() 一次只发一帧
 *     -> HAL_UART_TxCpltCallback：按实际帧长释放两个队列，再续发下一帧
 *   发送完成前不释放帧，保证 DMA 读取期间数据不被覆盖。
 *
 * 四、静态中转数组
 *   s_rxbuffer / s_txbuffer 按串口槽位分配，位于 .bss：
 *   - DMA 需要地址固定、生命周期贯穿运行期的内存；
 *   - 中断服务函数不能使用大局部数组（默认栈很小）；
 *   - 环形队列可能回绕，先拷到连续内存可保证“一帧 = 一次 DMA”。
 *
 * 五、并发
 *   队列同时被主循环（Transmit/Receive）与串口中断（RxEvent/TxCplt）访问，
 *   所有“检查 + 修改”都在 PRIMASK 临界区内完成。
 *
 * 六、限制
 *   - 单帧最大 UART_MAX（255）字节；如需更长帧，必须同时增大 UART_MAX、
 *     s_rxbuffer/s_txbuffer 与 UART_MAXNUM，RAM 占用会成倍增加；
 *   - IDLE 分帧依赖发送方帧内连续发送，帧内停顿会被拆帧。
 * ========================================================================
 */

#include "func_uart_0.h"
#include <string.h>

// 内部注册表：用于在中断回调中反查实例
static func_uart_base_t *s_uart_map[UART_NUM] = {NULL};
static uint8_t s_txbuffer_num[UART_NUM][UART_MAXNUM]; 
static uint8_t s_rxbuffer_num[UART_NUM][UART_MAXNUM];
static uint8_t s_rxbuffer[UART_NUM][UART_MAX];
static uint8_t s_txbuffer[UART_NUM][UART_MAX];

/* ==================== 临界区 ==================== */

/*
 * 队列同时被主循环与串口中断访问，所有“检查 + 修改”必须在临界区内完成。
 * 保存进入前的 PRIMASK，退出时原样恢复，嵌套调用也不会改变中断状态。
 */
static uint32_t Func_Uart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Func_Uart_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

//****************基础发送与接收**************************************************************//

static bool Func_Uart_BaseRx(UART_HandleTypeDef *huart, uint8_t *pData)
{
    //启动底层的 DMA 空闲接收
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart,pData, UART_MAX) != HAL_OK) {
        return false;
    }
    //关闭半满中断（防止重复触发，降低 CPU 开销）
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    return true;
}

static bool Func_Uart_BaseTx(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t length)
{
    //启动底层的 DMA 发送
    if (HAL_UART_Transmit_DMA(huart,pData,length) != HAL_OK) {
        return false;
    }

    return true;
}

//*****************************私有API*********************************************************/

/* 从长度队列窥视队首帧长（2 字节小端），不出队 */
static bool Func_Uart_PeekLength(circle_buffer_t *buffer_num, uint16_t *length)
{
    uint8_t low, high;

    if (buffer_num == NULL || length == NULL) return false;
    if (!Circle_Buffer_Peek(buffer_num, 0U, &low)) return false;
    if (!Circle_Buffer_Peek(buffer_num, 1U, &high)) return false;

    *length = ((uint16_t)high << 8) + (uint16_t)low;
    return true;
}

/* 把环形缓冲区队首一帧原样拷到连续内存，不出队 */
static void Func_Uart_CopyOut(const circle_buffer_t *buffer, uint8_t *out, uint16_t length)
{
    uint16_t first;

    if (buffer == NULL || out == NULL || length == 0U) return;

    first = (uint16_t)(buffer->length - buffer->head);
    if (first >= length) {
        (void)memcpy(out, &buffer->address[buffer->head], length);
    } else {
        (void)memcpy(out, &buffer->address[buffer->head], first);
        (void)memcpy(&out[first], buffer->address, (size_t)(length - first));
    }
}


static bool Func_Uart_Get(circle_buffer_t *buffer, circle_buffer_t *buffer_num,
                          uint8_t *frame, uint16_t capacity, uint16_t *out_length)
{
    uint16_t need_length;
    uint32_t primask;
    bool ok = false;

    if (out_length != NULL) *out_length = 0U;
    if (buffer == NULL || buffer_num == NULL || frame == NULL || capacity == 0U) {
        return false;
    }

    primask = Func_Uart_EnterCritical();

    if (Func_Uart_PeekLength(buffer_num, &need_length)) {
        if (need_length == 0U) {
            (void)Circle_Buffer_Drop(buffer_num, 2U);   /* 非法长度项，丢弃 */
        } else if ((need_length <= capacity) &&
                   (Circle_Buffer_Used(buffer) >= need_length)) {
            /*
             * 确认数据足够后再取；校验实际读出长度，成功后才 Drop 长度项，
             * 避免“长度已丢、数据没取到”造成两队列永久错位。
             */
            if (Circle_Buffer_Get_Buffer(buffer, frame, need_length) == need_length) {
                Circle_Buffer_Drop(buffer_num, 2U);
                if (out_length != NULL) *out_length = need_length;
                ok = true;
            }
        }
    }

    Func_Uart_ExitCritical(primask);
    return ok;
}

static bool Func_Uart_Put(circle_buffer_t *buffer, circle_buffer_t *buffer_num,
                          const uint8_t *frame, uint16_t length)
{
    uint8_t need_length[2];
    uint32_t primask;
    bool ok = false;

    if (buffer == NULL || buffer_num == NULL || frame == NULL || length == 0U) {
        return false;
    }

    need_length[0] = (uint8_t)length;
    need_length[1] = (uint8_t)(length >> 8);

    primask = Func_Uart_EnterCritical();

    if ((Circle_Buffer_Free(buffer_num) >= 2U) &&
        (Circle_Buffer_Free(buffer) >= length)) {
        /*
         * 临界区内空间已确认足够，Put_Buffer 不会只写一半。
         * 先写数据、后写长度：任何时刻看到长度项时，数据一定已经就绪。
         */
        (void)Circle_Buffer_Put_Buffer(buffer, frame, length);
        (void)Circle_Buffer_Put_Buffer(buffer_num, need_length, 2U);
        ok = true;
    }

    Func_Uart_ExitCritical(primask);
    return ok;
}

/*
 * 若发送空闲：从长度队列窥视队首帧长，把数据队列中的这一帧拷入
 * 静态中转数组 s_txbuffer[slot]，再启动一次 DMA。
 * 发送完成前不 Drop，由 TxCplt 释放。
 */
static bool Func_Uart_StartTx(func_uart_base_t *dev)
{
    uint16_t frame_len;
    uint32_t primask;

    if (dev == NULL || dev->huart == NULL || dev->slot >= UART_NUM) return false;

    primask = Func_Uart_EnterCritical();

    if (dev->tx_busy) {                         /* 上一帧还在发送 */
        Func_Uart_ExitCritical(primask);
        return true;
    }

    if (!Func_Uart_PeekLength(&dev->txbuffer_num, &frame_len)) {
        Func_Uart_ExitCritical(primask);
        return true;                            /* 没有待发送帧 */
    }

    if ((frame_len == 0U) || (frame_len > UART_MAX)) {
        (void)Circle_Buffer_Drop(&dev->txbuffer_num, 2U);
        Func_Uart_ExitCritical(primask);
        return false;                           /* 长度异常，丢弃该项 */
    }

    if (Circle_Buffer_Used(&dev->txbuffer) < frame_len) {
        Func_Uart_ExitCritical(primask);
        return false;                           /* 数据尚未写齐 */
    }

    Func_Uart_CopyOut(&dev->txbuffer, s_txbuffer[dev->slot], frame_len);

    /* 先占位再启动 DMA，防止主循环与 TxCplt 同时启动两帧 */
    dev->tx_frame_len = frame_len;
    dev->tx_busy      = true;

    Func_Uart_ExitCritical(primask);

    if (!Func_Uart_BaseTx(dev->huart, s_txbuffer[dev->slot], frame_len)) {
        primask = Func_Uart_EnterCritical();
        dev->tx_frame_len = 0U;
        dev->tx_busy      = false;
        Func_Uart_ExitCritical(primask);
        return false;                           /* 启动失败，帧仍在队列中 */
    }

    return true;
}


//**************************************公共API******************************************************//

bool Func_Uart_Init(func_uart_base_t *dev, uint8_t *rxbuffer, uint16_t rx_length,
                    uint8_t *txbuffer, uint16_t tx_length,
                    UART_HandleTypeDef *huart)
{
    // 1. 防御性检查
    if (dev == NULL || rxbuffer == NULL || txbuffer == NULL || huart == NULL) return false;
    if (rx_length == 0 || tx_length == 0) return false;

    // 2. 检查此 huart 是否已被注册，且同一个 dev 不允许重复初始化
    for (int i = 0; i < UART_NUM; i++) {
        if ((s_uart_map[i] == dev) ||
            (s_uart_map[i] != NULL && s_uart_map[i]->huart == huart)) {
            return false;
        }
    }

    // 3. 绑定用户传入的缓冲区和句柄
    dev->huart = huart;
    dev->rxbuffer = Circle_Buffer_Init(rxbuffer, rx_length);
    dev->txbuffer = Circle_Buffer_Init(txbuffer, tx_length);

    // 4. 注册到静态数组中（找空位存放 dev 指针）
    bool registered = false;
    int i = 0;
    for (; i < UART_NUM; i++) {
        if (s_uart_map[i] == NULL) {
            s_uart_map[i] = dev;
            registered = true;

            dev->txbuffer_num = Circle_Buffer_Init(s_txbuffer_num[i],UART_MAXNUM);
            dev->rxbuffer_num = Circle_Buffer_Init(s_rxbuffer_num[i],UART_MAXNUM);
            dev->slot         = (uint8_t)i;
            dev->rx_drop_cnt  = 0U;
            dev->tx_busy      = false;
            dev->tx_frame_len = 0U;
            dev->err_cnt      = 0U;
            dev->initialized  = true;

            break;
        }
    }
    if (!registered) return false; // 槽位已满

    if (!Func_Uart_BaseRx(huart, s_rxbuffer[i])) {
        s_uart_map[i] = NULL;   /* 启动失败：回滚注册，避免槽位泄漏 */
        dev->initialized = false;
        return false;
    }

    return true;
}

bool Func_Uart_Transmit(func_uart_base_t *dev, const uint8_t *frame, uint16_t length)
{
    if (dev == NULL || !dev->initialized || frame == NULL ||
        length == 0U || length > UART_MAX) {
        return false;
    }

    if (!Func_Uart_Put(&dev->txbuffer, &dev->txbuffer_num, frame, length)) {
        return false;               /* 空间不足：整帧丢弃 */
    }

    (void)Func_Uart_StartTx(dev);   /* 空闲时立即发送，忙时排队 */
    return true;
}


bool Func_Uart_Receive(func_uart_base_t *dev, uint8_t *frame, uint16_t length, uint16_t *out_length)
{
    if (dev == NULL || !dev->initialized) return false;

    return Func_Uart_Get(&dev->rxbuffer, &dev->rxbuffer_num, frame, length, out_length);
}

void Func_Uart_DeInit(func_uart_base_t *dev)
{
    uint32_t primask;

    if (dev == NULL || !dev->initialized) return;

    /* 先停硬件（临界区外，避免 HAL 内部等待时被关中断影响） */
    (void)HAL_UART_AbortReceive(dev->huart);
    if (dev->tx_busy) {
        (void)HAL_UART_AbortTransmit(dev->huart);
    }

    primask = Func_Uart_EnterCritical();

    for (int i = 0; i < UART_NUM; i++) {
        if (s_uart_map[i] == dev) {
            s_uart_map[i] = NULL;
            break;
        }
    }

    dev->tx_busy      = false;
    dev->tx_frame_len = 0U;
    dev->initialized  = false;

    Func_Uart_ExitCritical(primask);
}

//**************************************中断回调函数******************************************************//

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    func_uart_base_t *dev = NULL;

    for (int i = 0; i < UART_NUM; i++) {
        if (s_uart_map[i] != NULL && s_uart_map[i]->huart == huart) {
            dev = s_uart_map[i];
            break;
        }
    }
    if (dev == NULL) return;

    /* 防御：DMA 长度固定为 UART_MAX，Size 不应超过它 */
    if (Size > UART_MAX) Size = UART_MAX;

    /* 半满事件不是帧边界：不提交、不重启，继续等待 IDLE/TC */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    /*
     * IDLE / TC：本轮 DMA 收到的整段数据即一帧。
     * 提交到数据队列和长度队列；任一处空间不足都整帧丢弃，
     * 绝不写入半帧，避免长度队列与数据队列错位。
     */
    if (Size > 0U) {
        if (!Func_Uart_Put(&dev->rxbuffer, &dev->rxbuffer_num,
                           s_rxbuffer[dev->slot], Size)) {
            dev->rx_drop_cnt++;
        }
    }

    /* 本轮结束，重启 DMA 等待下一帧 */
    (void)Func_Uart_BaseRx(huart, s_rxbuffer[dev->slot]);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    func_uart_base_t *dev = NULL;
    uint16_t frame_len;
    uint32_t primask;

    for (int i = 0; i < UART_NUM; i++) {
        if (s_uart_map[i] != NULL && s_uart_map[i]->huart == huart) {
            dev = s_uart_map[i];
            break;
        }
    }
    if (dev == NULL) return;

    primask = Func_Uart_EnterCritical();

    /* 这一帧已经真正发完，现在才释放两个队列中的对应记录 */
    frame_len = dev->tx_frame_len;
    dev->tx_frame_len = 0U;
    dev->tx_busy      = false;

    if (frame_len > 0U) {
        Circle_Buffer_Drop(&dev->txbuffer, frame_len);
        Circle_Buffer_Drop(&dev->txbuffer_num, 2U);
    }

    Func_Uart_ExitCritical(primask);

    (void)Func_Uart_StartTx(dev);   /* 续发下一帧；队列为空则停止 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    func_uart_base_t *dev = NULL;
    uint16_t pending;
    uint32_t primask;

    for (int i = 0; i < UART_NUM; i++) {
        if (s_uart_map[i] != NULL && s_uart_map[i]->huart == huart) {
            dev = s_uart_map[i];
            break;
        }
    }
    if (dev == NULL) return;

    primask = Func_Uart_EnterCritical();

    dev->err_cnt++;

    /* 发送被打断：这一帧可能只发出去一部分，丢弃，避免下一帧错位 */
    pending = dev->tx_frame_len;
    dev->tx_frame_len = 0U;
    dev->tx_busy      = false;
    if (pending > 0U) {
        Circle_Buffer_Drop(&dev->txbuffer, pending);
        Circle_Buffer_Drop(&dev->txbuffer_num, 2U);
    }

    Func_Uart_ExitCritical(primask);

    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_AbortTransmit(huart);
    (void)Func_Uart_BaseRx(huart, s_rxbuffer[dev->slot]);
    (void)Func_Uart_StartTx(dev);
}
