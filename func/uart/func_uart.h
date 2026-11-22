#ifndef FUNC_UART_H
#define FUNC_UART_H

#include "main.h"
#include "circle_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 最多支持同时管理的串口个数（决定静态数组和注册表的维度） */
#define UART_NUM 4

/*
 * 单帧最大长度（含包头和包尾），同时也是三块静态中转数组的大小。
 * 如果协议帧更长，需要同时增大该宏；发送和接收都会被这个上限约束。
 */
#define UART_SIZE 255

/* 接收组帧状态机 */
typedef enum {
    FUNC_UART_FRAME_IDLE = 0,   /* 等待包头：当前字节不是包头就丢弃 */
    FUNC_UART_FRAME_BUSY        /* 已收到包头：正在把后续字节拼进组帧数组 */
} func_uart_frame_state_t;

/*
 * ============================ 模块原理 ============================
 *
 * 一、帧格式（收和发使用同一约定）：
 *
 *     +--------+--------------------------------+--------+
 *     |  包头  |            payload             |  包尾  |
 *     +--------+--------------------------------+--------+
 *       1 字节       0 ~ (UART_SIZE - 2) 字节      1 字节
 *
 *    约束：payload 中间不能出现包尾字节，否则会被当作帧结束；
 *          包头字节可以出现在 payload 中（接收状态机在 BUSY 状态下只认包尾）。
 *
 * 二、接收数据流：
 *
 *     串口 RX 引脚
 *        │  ① DMA 硬件逐字节搬运，CPU 不参与
 *        ▼
 *     s_dma_rx_buf[UART_NUM][UART_SIZE]     静态接收中转数组
 *        │  ② IDLE / TC 中断：只把“本次新增的字节”交给状态机
 *        ▼
 *     组帧状态机 + s_frame_buf              静态组帧数组
 *        │  ③ 找到包头 → 拼帧 → 遇到包尾
 *        │     整帧完整且 rxbuffer 空间足够：整帧写入
 *        │     空间不足或帧超长：整帧丢弃（绝不写半帧）
 *        ▼
 *     rxbuffer（环形缓冲区，只保存完整帧）
 *        │  ④ Func_Uart_Receive() 一次取出一帧
 *        ▼
 *     用户程序
 *
 * 三、发送数据流：
 *
 *     用户程序
 *        │  ① Func_Uart_Transmit() 只接受完整帧，整帧原子入队
 *        ▼
 *     txbuffer（环形缓冲区，只保存完整帧）
 *        │  ② 空闲时从队首按“包头 → 第一个包尾”确定一帧
 *        │     整帧拷入连续内存（解决环形回绕）
 *        ▼
 *     s_dma_tx_buf[UART_NUM][UART_SIZE]     静态发送中转数组
 *        │  ③ 一次 DMA 只发这一帧
 *        ▼
 *     串口 TX 引脚
 *        │  ④ 发送完成（TC）中断：释放这一帧，再取下一帧
 *        ▼
 *     （队列为空则停止）
 *
 * 四、为什么要用静态数组：
 *   - DMA 需要一个地址固定的内存区，静态数组的生命周期贯穿整个运行期，
 *     不会像局部变量那样在函数返回后失效；
 *   - 中断服务函数不能使用大数组（默认栈很小），静态数组放在 .bss 中；
 *   - 发送环形缓冲区可能回绕，一帧可能被数组末尾截成两段；先拷进静态
 *     连续中转数组，就能保证“一帧 = 一次 DMA”，实现和时序都更直观。
 *
 * 五、为什么要用环形缓冲区：
 *   串口收发速度与主循环处理速度不匹配。中断只负责把数据搬进/搬出缓冲，
 *   主循环按自己的节奏取帧/投帧，两边通过 head/tail/count 解耦；
 *   空间不足时选择丢弃新数据而不是覆盖旧数据，避免破坏已缓存的完整帧。
 */

typedef struct {
    /* ---- 用户初始化参数（由 Func_Uart_Init 填写） ---- */
    circle_buffer_t rxbuffer;       /* 接收环形缓冲区：只存放完整帧（含包头包尾） */
    circle_buffer_t txbuffer;       /* 发送环形缓冲区：只存放完整帧（含包头包尾） */
    uint8_t frame_head;             /* 包头字符 */
    uint8_t frame_tail;             /* 包尾字符 */
    UART_HandleTypeDef *huart;      /* 串口句柄，回调中用它反查实例 */

    /* ---- 以下状态由模块内部维护，请勿手动修改 ---- */

    /* 接收侧：DMA 与组帧 */
    uint8_t *dma_rx_buf;            /* 指向静态接收中转数组 s_dma_rx_buf[slot] */
    uint16_t dma_rx_size;          /* 本轮 DMA 的接收长度（固定为 UART_SIZE） */
    uint16_t dma_rx_processed;     /* 本轮 DMA 中已解析到的偏移，防止 HT/IDLE/TC 重复解析 */

    uint8_t *frame_buf;             /* 指向静态组帧数组 s_frame_buf[slot] */
    uint16_t frame_buf_size;       /* 组帧数组容量（UART_SIZE） */
    uint16_t frame_len;            /* 当前正在组装的帧已收字节数 */
    func_uart_frame_state_t frame_state; /* 组帧状态：等包头 / 收帧中 */
    bool frame_overflow;            /* 当前帧超长，收包尾后整帧丢弃 */

    /* 发送侧：一帧一次 DMA */
    uint8_t *dma_tx_buf;            /* 指向静态发送中转数组 s_dma_tx_buf[slot] */
    uint16_t tx_frame_len;         /* 当前 DMA 正在发送的帧长度；发送完成前不释放 */
    bool tx_busy;                   /* true 表示 DMA 正在发送一帧 */
    bool initialized;               /* 实例是否已注册 */

    /* ---- 运行统计（便于调试，不影响收发逻辑） ---- */
    uint32_t rx_drop_cnt;           /* 因空间不足/超长而丢弃的接收帧数 */
    uint32_t tx_drop_cnt;           /* 因发送缓冲区空间不足而丢弃的帧数 */
    uint32_t err_cnt;               /* 底层 HAL 启动/恢复失败次数 */
} func_uart_base_t;

/*
 * 初始化一个串口实例。
 *
 * 参数：
 *   uart_dev   实例结构体（通常定义为全局/静态变量）
 *   rxbuffer   接收环形缓冲区的实体数组（由调用者分配）
 *   rx_length  接收缓冲区容量
 *   txbuffer   发送环形缓冲区的实体数组（由调用者分配）
 *   tx_length  发送缓冲区容量
 *   frame_head 包头字节
 *   frame_tail 包尾字节
 *   huart      串口句柄（如 &huart1）
 *
 * 内部动作：
 *   1. 在注册表中占用一个槽位，并把该槽位对应的三块静态数组绑定到实例；
 *   2. 清空环形缓冲区与状态机；
 *   3. 启动 HAL_UARTEx_ReceiveToIdle_DMA()，进入 DMA 空闲接收状态。
 *
 * 注意：必须在 MX_USARTx_UART_Init() 和 MX_DMA_Init() 之后调用。
 * 返回 true 表示初始化成功；失败时不会留下半注册状态。
 */
bool Func_Uart_Init(func_uart_base_t *uart_dev,
                    uint8_t *rxbuffer, uint16_t rx_length,
                    uint8_t *txbuffer, uint16_t tx_length,
                    uint8_t frame_head, uint8_t frame_tail,
                    UART_HandleTypeDef *huart);

/*
 * 注销实例：
 *   中止该串口的 DMA 收发，从注册表移除。
 *   注销后 HAL 回调再也找不到该实例，不会误操作已失效的结构体。
 */
void Func_Uart_DeInit(func_uart_base_t *uart_dev);

/*
 * 发送一帧（只接受完整帧）。
 *
 * 合法性检查（任一不满足都返回 false，不入队、不发 DMA）：
 *   - 2 <= length <= UART_SIZE；
 *   - frame[0] == 包头；
 *   - frame[length - 1] == 包尾；
 *   - payload（frame[1] ~ frame[length-2]）中不得出现包尾字节。
 *
 * 空间策略：
 *   - txbuffer 剩余空间 >= length：整帧写入，返回 true；
 *   - 剩余空间不足：整帧丢弃（不写任何字节），tx_drop_cnt++，返回 false。
 *
 * 发送时序：
 *   - 若当前空闲，立即把队首这一帧拷入静态中转数组并发起一次 DMA；
 *   - 若上一帧仍在发送，本帧只排队，等发送完成回调处理后才会发出；
 *   - 因此一帧对应一次 DMA，帧与帧不会合并。
 */
bool Func_Uart_Transmit(func_uart_base_t *uart_dev, const uint8_t *frame, uint16_t length);

/*
 * 取出接收缓冲区中的一帧（含包头包尾）。
 *
 * 参数：
 *   uart_dev    实例
 *   frame       输出缓冲区
 *   length      frame 的容量
 *   out_length  成功时写入帧长度（含包头包尾），可为 NULL
 *
 * 行为：
 *   - 一次只取一帧，不会把两帧拼在一起；
 *   - 没有完整帧时返回 false；
 *   - 队首帧比 length 长时返回 false 且不消费该帧，
 *     调用者需要提供足够大的缓冲区（最大 UART_SIZE）。
 */
bool Func_Uart_Receive(func_uart_base_t *uart_dev, uint8_t *frame, uint16_t length, uint16_t *out_length);

#ifdef __cplusplus
}
#endif

#endif /* FUNC_UART_H */
