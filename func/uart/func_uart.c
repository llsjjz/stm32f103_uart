#include "func_uart.h"
#include <string.h>

/*
 * ============================ 实现原理 ============================
 *
 * 本文件只对外提供四个接口：
 *   Func_Uart_Init / Func_Uart_DeInit / Func_Uart_Transmit / Func_Uart_Receive
 *
 * ------------------------------------------------------------------
 * 一、接收：DMA -> 静态中转 -> 组帧 -> 环形缓冲区 -> 用户
 * ------------------------------------------------------------------
 *   1. Func_Uart_Init() 调用 HAL_UARTEx_ReceiveToIdle_DMA()：
 *        - DMA 工作在 Normal 模式，串口每收到一个字节，硬件自动写入
 *          s_dma_rx_buf，CPU 完全不参与；
 *        - 同时使能 USART 的 IDLE 中断和 DMA 传输完成（TC）中断。
 *   2. 一轮接收可能产生三种事件，都会进入 HAL_UARTEx_RxEventCallback()：
 *        - IDLE：线路出现空闲（一帧发完后发送方停顿时最容易出现）；
 *        - TC  ：DMA 把 UART_SIZE 字节的中转数组写满了；
 *        - HT  ：DMA 写满一半（本模块在启动时主动关闭了它，减少中断）。
 *
 *      HAL 回调里的 Size 表示“本轮 DMA 启动以来累计收到的字节数”：
 *        - IDLE 事件：Size = 目前实际收到的字节数（可能不满一缓冲区）；
 *        - TC  事件：Size = UART_SIZE（缓冲区已满）；
 *        - HT  事件：Size = UART_SIZE / 2。
 *      如果用 Size 直接覆盖处理，HT 之后再来 IDLE/TC 会把前半段重复解析。
 *      为此用 dma_rx_processed 记录“已经解析到哪个偏移”，每次只处理
 *      [processed, Size) 这段新增数据。
 *   3. 每次只处理“本轮结束”的事件（IDLE / TC）之后，重新启动 DMA；
 *      HT 事件不结束本轮接收，所以只解析、不重启。
 *   4. 组帧状态机 Func_Uart_RxParse()：
 *        - IDLE 状态：逐字节丢弃，直到遇到包头，转入 BUSY；
 *        - BUSY 状态：把字节存入 s_frame_buf；遇到包尾表示一帧结束。
 *        - payload 中的包头字节会被当作普通数据（BUSY 状态只认包尾）；
 *        - 帧长超过组帧数组时标记 overflow，继续扫描到包尾后整帧丢弃，
 *          这样不会因为一个坏帧把后面的好帧也卡住。
 *   5. Func_Uart_CommitFrame()：
 *        - 只有“整帧完整”且“rxbuffer 剩余空间 >= 帧长”时才整帧写入；
 *        - 否则整帧丢弃（rx_drop_cnt++），绝不写入半帧——
 *          否则用户取到的帧头尾会对不上。
 *
 * ------------------------------------------------------------------
 * 二、发送：用户 -> 环形缓冲区 -> 静态中转 -> 一次 DMA 发一帧
 * ------------------------------------------------------------------
 *   1. Func_Uart_Transmit() 只接受完整帧，并做四项校验：
 *        长度合法、首字节是包头、末字节是包尾、payload 中不含包尾。
 *      最后一条很重要：发送端靠“队首到第一个包尾”确定帧边界，
 *      payload 里若有包尾会被误认为帧结束，导致后面的字节错位。
 *   2. 整帧原子写入 txbuffer：
 *        - 剩余空间不足 -> 整帧丢弃，一个字节都不写；
 *        - 这就是“空间不够就放弃新数据”的策略在发送侧的实现。
 *   3. 发送空闲时（tx_busy == false）：
 *        - 从 txbuffer 队首开始，找到第一个包尾，确定这一帧长度；
 *        - 把整帧拷贝到静态中转数组 s_dma_tx_buf；
 *        - 调 HAL_UART_Transmit_DMA() 发这一帧，记录 tx_frame_len。
 *      为什么要拷贝：环形缓冲区是一个环，一帧可能从数组末尾绕回开头，
 *      物理上被分成两段；拷到连续数组后，一帧只需一次 DMA，边界清晰。
 *   4. 发送完成的判定：DMA 只负责把字节搬到 USART 数据寄存器，
 *      最后一个字节的停止位真正移出引脚后，USART 的 TC 标志才置位，
 *      HAL 此时才调用 HAL_UART_TxCpltCallback()。
 *      在回调里才 Circle_Buffer_Drop() 释放这一帧的缓冲区空间，
 *      保证 DMA 读取期间这块数据不会被新数据覆盖。
 *   5. 释放后若队列里还有完整帧，立刻发下一帧；队列为空则 tx_busy 清零，
 *      等下一次 Func_Uart_Transmit()。因此同一时刻线上最多一帧，
 *      不会出现“两帧合并成一次 DMA”的情况。
 *
 * ------------------------------------------------------------------
 * 三、并发与临界区
 * ------------------------------------------------------------------
 *   共享数据的访问者：
 *      rxbuffer：RX 中断写（提交整帧），主循环读（Func_Uart_Receive）；
 *      txbuffer：主循环写（整帧入队），TX 完成中断读并释放队首帧；
 *      tx_busy / tx_frame_len / dma_rx_processed：主循环与中断都会读写。
 *   所有“先检查条件、再修改数据”的复合操作都放进 PRIMASK 临界区：
 *      __disable_irq() 屏蔽所有可屏蔽中断，退出时恢复进入前的 PRIMASK，
 *      所以即使函数本身在中断里被调用（嵌套临界区）也能正确恢复。
 *   临界区都很短（最坏是一次 <= UART_SIZE 的 memcpy），不影响串口实时性。
 *   注意：HAL 的 Abort/启动类调用可能内部等待 DMA 标志，不放在临界区内，
 *         避免在关中断状态下等待 SysTick 超时而卡死。
 */

/* ==================== 静态中转数组与实例表 ==================== */

/*
 * 按“串口槽位”分配，而不是按实例分配：
 *   每个 Func_Uart_Init() 成功的实例占用一个槽位 slot，
 *   该串口的 RX DMA 落点、组帧缓冲、TX 中转缓冲分别使用三块静态数组的第 slot 行。
 *   静态数组位于 .bss，生命周期贯穿整个程序，可安全交给 DMA。
 */
static uint8_t s_dma_rx_buf[UART_NUM][UART_SIZE];   /* 接收 DMA 落点 */
static uint8_t s_frame_buf[UART_NUM][UART_SIZE];    /* 接收组帧缓冲：拼出完整一帧 */
static uint8_t s_dma_tx_buf[UART_NUM][UART_SIZE];   /* 发送中转缓冲：一帧一次连续 DMA */

/*
 * 注册表：HAL 回调只拿到 huart 指针，通过线性查找找到对应的 func_uart_base_t。
 * 槽位为 NULL 表示空闲；注销时置 NULL。
 */
static func_uart_base_t *s_uart_map[UART_NUM];

/* ==================== 临界区 ==================== */

/*
 * 进入临界区：保存当前 PRIMASK（中断是否已关闭），然后关闭所有可屏蔽中断。
 * 返回旧值，退出时必须原样写回，这样嵌套调用不会把中断状态改乱。
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

/* ==================== 内部私有函数 ==================== */

/* 通过 huart 指针在注册表中查找实例；回调中反复使用 */
static func_uart_base_t *Func_Uart_Find(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < UART_NUM; i++) {
        if ((s_uart_map[i] != NULL) && (s_uart_map[i]->huart == huart)) {
            return s_uart_map[i];
        }
    }
    return NULL;
}

/*
 * 复位组帧状态机。
 * 用于初始化、注销、清空和错误恢复：丢弃当前半帧，重新等待包头。
 */
static void Func_Uart_FrameReset(func_uart_base_t *dev)
{
    dev->frame_state    = FUNC_UART_FRAME_IDLE;
    dev->frame_len      = 0U;
    dev->frame_overflow = false;
}

/*
 * 启动（或重启）DMA 空闲接收。
 *
 * 调用时机：
 *   - 初始化时；
 *   - 每次 IDLE / TC 事件处理完数据后。
 *
 * 失败重试：
 *   如果 HAL 返回 HAL_BUSY（上一轮接收没有完全停止），先 Abort 再启动一次；
 *   仍失败则 err_cnt++ 并返回 false，等待下一次错误回调或用户重新初始化。
 *
 * 关闭 HT 中断：
 *   ReceiveToIdle_DMA 默认同时打开半满中断。本模块只关心 IDLE/TC，
 *   关闭 HT 可以减少中断次数；即使它仍然触发，回调里也有 processed 偏移保护。
 */
static bool Func_Uart_StartRx(func_uart_base_t *dev)
{
    if ((dev == NULL) || (dev->huart == NULL) ||
        (dev->dma_rx_buf == NULL) || (dev->dma_rx_size == 0U)) {
        return false;
    }

    /* 新的一轮 DMA 从偏移 0 开始，先清零，防止上一轮的偏移残留 */
    dev->dma_rx_processed = 0U;

    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(dev->huart, dev->dma_rx_buf, dev->dma_rx_size);

    if (status != HAL_OK) {
        /* 可能是上一次接收未完全停止：强制 Abort 后重试一次 */
        (void)HAL_UART_AbortReceive(dev->huart);
        dev->dma_rx_processed = 0U;
        status = HAL_UARTEx_ReceiveToIdle_DMA(dev->huart, dev->dma_rx_buf, dev->dma_rx_size);
    }

    if (status != HAL_OK) {
        dev->err_cnt++;
        return false;
    }

    /* 只关心 IDLE / TC，关闭半满中断以减少中断开销 */
    if (dev->huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(dev->huart->hdmarx, DMA_IT_HT);
    }
    return true;
}

/*
 * 把组帧数组中的一整帧提交到接收环形缓冲区。
 *
 * 空间判断是“整帧”级别的：
 *   剩余空间 >= frame_len  才写入；
 *   否则整帧丢弃，绝不写入半帧，保证 rxbuffer 中每个元素都是完整帧。
 *
 * 这里运行在 RX 中断上下文；主循环的取出操作用临界区保护，
 * 因此本函数内部不需要再加锁（中断不会被主循环打断）。
 */
static void Func_Uart_CommitFrame(func_uart_base_t *dev)
{
    if (Circle_Buffer_Free(&dev->rxbuffer) < dev->frame_len) {
        dev->rx_drop_cnt++;             /* 空间不够，放弃这一帧新数据 */
        return;
    }

    (void)Circle_Buffer_Put_Buffer(&dev->rxbuffer, dev->frame_buf, dev->frame_len);
}

/*
 * 接收组帧状态机：对“本轮新增的字节”逐个扫描。
 *
 * 状态转换：
 *   IDLE --收到包头--> BUSY
 *   BUSY --收到包尾--> 提交整帧 --> IDLE
 *
 * 超长处理：
 *   frame_len 达到组帧数组容量后不再写入（防止越界），只置 overflow；
 *   继续扫描直到包尾，然后整帧丢弃并回到 IDLE。
 *   这样即使收到一个永远不结束的“假包头”，也会在下一个包尾处恢复同步。
 *
 * 注意：包头字节在 BUSY 状态下不做特殊处理，直接作为 payload 保存；
 *       只有当帧处于 IDLE 状态时，包头才具有“帧开始”的含义。
 */
static void Func_Uart_RxParse(func_uart_base_t *dev, const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++) {
        uint8_t byte = data[i];

        if (dev->frame_state == FUNC_UART_FRAME_IDLE) {
            /* 等待包头：其他字节一律视为帧间噪声丢弃 */
            if (byte == dev->frame_head) {
                dev->frame_state    = FUNC_UART_FRAME_BUSY;
                dev->frame_len      = 1U;
                dev->frame_overflow = false;
                dev->frame_buf[0]   = byte;
            }
            continue;
        }

        /* 已收到包头：先存入组帧数组（超过容量只标记，不越界写） */
        if (dev->frame_len < dev->frame_buf_size) {
            dev->frame_buf[dev->frame_len] = byte;
            dev->frame_len++;
        } else {
            dev->frame_overflow = true;
        }

        /* 包尾表示一帧结束；完整且未超长才提交 */
        if (byte == dev->frame_tail) {
            if (!dev->frame_overflow) {
                Func_Uart_CommitFrame(dev);
            } else {
                dev->rx_drop_cnt++;
            }
            Func_Uart_FrameReset(dev);
        }
    }
}

/*
 * 在接收缓冲区中定位队首这一帧：返回帧长（含包头包尾），找不到返回 0。
 *
 * 为什么从偏移 1 开始找包尾：
 *   rxbuffer 中只保存完整帧，第 0 字节必然是包头，
 *   真正的包尾最早只能出现在第 1 字节（长度 2 的空 payload 帧）。
 *
 * 为什么可以用“第一个包尾”作为帧尾：
 *   入队时已保证 payload 中不含包尾（接收状态机在遇到第一个包尾时就截断了），
 *   因此队首帧的边界是唯一确定的。
 */
static uint16_t Func_Uart_FindRxFrame(const func_uart_base_t *dev)
{
    uint16_t used = Circle_Buffer_Used(&dev->rxbuffer);
    if (used < 2U) return 0U;

    for (uint16_t i = 1U; i < used; i++) {
        uint8_t byte;
        if (Circle_Buffer_Peek(&dev->rxbuffer, i, &byte) && (byte == dev->frame_tail)) {
            return (uint16_t)(i + 1U);
        }
    }
    return 0U;
}

/*
 * 从发送环形缓冲区取第一帧，拷入静态发送中转数组并启动一次 DMA。
 *
 * 前置条件：调用者必须已经持有临界区（见函数名中的 Locked）。
 * 这样做是因为发送完成中断和主循环都可能触发发送，
 * “检查 tx_busy / 找帧长 / 拷贝 / 启动 DMA / 更新 tx_busy”必须是一个原子序列，
 * 否则两个上下文可能同时启动 DMA 或重复释放同一帧。
 *
 * 状态变量：
 *   tx_frame_len：本次 DMA 发送的帧长度。发送完成前不释放 txbuffer 空间；
 *                 完成回调根据它 Drop 队首这一帧。
 *   tx_busy     ：有 DMA 在发送中。为 true 时不再启动新的 DMA。
 *
 * 为什么不直接把 txbuffer 的地址交给 DMA：
 *   环形缓冲区可能回绕，一帧在物理上可能是“数组末尾一段 + 数组开头一段”。
 *   先拷贝到 s_dma_tx_buf（连续内存）即可保证一帧一次 DMA，
 *   同时避免 DMA 读取期间主循环写入导致数据被覆盖。
 */
static bool Func_Uart_TxStartLocked(func_uart_base_t *dev)
{
    if (dev->tx_busy) {
        return true;                    /* 上一帧还在发送，等发送完成回调再续 */
    }

    uint16_t used = Circle_Buffer_Used(&dev->txbuffer);
    if (used < 2U) {
        return true;                    /* 没有待发送的完整帧（最短帧也要 2 字节） */
    }

    /*
     * 队首一定是某个帧的包头（入队时按整帧写入）。
     * 从偏移 1 开始找第一个包尾，就得到队首这一帧的长度。
     * 入队时已禁止 payload 中出现包尾，因此这里不会提前截断。
     */
    uint16_t frame_len = 0U;
    for (uint16_t i = 1U; i < used; i++) {
        uint8_t byte;
        if (Circle_Buffer_Peek(&dev->txbuffer, i, &byte) && (byte == dev->frame_tail)) {
            frame_len = (uint16_t)(i + 1U);
            break;
        }
    }

    /* 正常不会出现无包尾或超长的情况（入队时已校验），防御性处理 */
    if ((frame_len < 2U) || (frame_len > UART_SIZE)) {
        return false;
    }

    /*
     * 整帧拷贝到连续内存：
     *   first = 从 head 到数组末尾的剩余长度。
     *   如果这一帧没有跨过末尾，一次 memcpy 即可；
     *   否则拆成“末尾一段 + 开头一段”两次 memcpy。
     * 拷完之后这一帧在 s_dma_tx_buf[0 .. frame_len-1] 中连续存放。
     */
    uint16_t first = (uint16_t)(dev->txbuffer.length - dev->txbuffer.head);
    if (first >= frame_len) {
        (void)memcpy(dev->dma_tx_buf, &dev->txbuffer.address[dev->txbuffer.head], frame_len);
    } else {
        (void)memcpy(dev->dma_tx_buf, &dev->txbuffer.address[dev->txbuffer.head], first);
        (void)memcpy(&dev->dma_tx_buf[first], dev->txbuffer.address,
                     (size_t)(frame_len - first));
    }

    /* 一次 DMA 只发这一帧；HAL 返回 OK 后记录长度并置忙 */
    HAL_StatusTypeDef status =
        HAL_UART_Transmit_DMA(dev->huart, dev->dma_tx_buf, frame_len);

    if (status == HAL_OK) {
        dev->tx_frame_len = frame_len;
        dev->tx_busy      = true;
    } else {
        dev->err_cnt++;                 /* 启动失败：帧仍留在队列里，下次重试 */
    }
    return (status == HAL_OK);
}

/* 自行进入临界区的发送启动版本，供主循环与错误恢复路径调用 */
static bool Func_Uart_TxStart(func_uart_base_t *dev)
{
    if ((dev == NULL) || !dev->initialized || (dev->huart == NULL)) return false;

    uint32_t primask = Func_Uart_EnterCritical();
    bool ok = Func_Uart_TxStartLocked(dev);
    Func_Uart_ExitCritical(primask);
    return ok;
}

/* ==================== 对外公开的 API ==================== */

bool Func_Uart_Init(func_uart_base_t *uart_dev,
                    uint8_t *rxbuffer, uint16_t rx_length,
                    uint8_t *txbuffer, uint16_t tx_length,
                    uint8_t frame_head, uint8_t frame_tail,
                    UART_HandleTypeDef *huart)
{
    if ((uart_dev == NULL) || (rxbuffer == NULL) || (txbuffer == NULL) ||
        (rx_length == 0U) || (tx_length == 0U) || (huart == NULL)) {
        return false;
    }

    int slot = -1;

    uint32_t primask = Func_Uart_EnterCritical();

    /*
     * 槽位分配：
     *   - 如果该实例已经注册过，复用原槽位（便于重新初始化）；
     *   - 否则使用第一个空闲槽位；
     *   - 全部占满则初始化失败。
     * 三个静态数组的行号与槽位一一对应。
     */
    for (uint32_t i = 0U; i < UART_NUM; i++) {
        if (s_uart_map[i] == uart_dev) {
            slot = (int)i;
            break;
        }
        if ((s_uart_map[i] == NULL) && (slot < 0)) {
            slot = (int)i;
        }
    }

    if (slot < 0) {                     /* 没有空闲槽位 */
        Func_Uart_ExitCritical(primask);
        return false;
    }

    /* 同一个 huart 不允许注册到两个不同实例，否则回调无法确定归属 */
    for (uint32_t i = 0U; i < UART_NUM; i++) {
        if ((s_uart_map[i] != NULL) && (s_uart_map[i] != uart_dev) &&
            (s_uart_map[i]->huart == huart)) {
            Func_Uart_ExitCritical(primask);
            return false;
        }
    }

    /* 清零后逐项填写，避免残留旧状态（重新初始化时尤其重要） */
    (void)memset(uart_dev, 0, sizeof(*uart_dev));

    /* 两个环形缓冲区绑定用户提供的数组，只存完整帧 */
    uart_dev->rxbuffer         = Circle_Buffer_Init(rxbuffer, rx_length);
    uart_dev->txbuffer         = Circle_Buffer_Init(txbuffer, tx_length);
    uart_dev->frame_head       = frame_head;
    uart_dev->frame_tail       = frame_tail;
    uart_dev->huart            = huart;

    /* 三块静态数组按槽位绑定给实例 */
    uart_dev->dma_rx_buf       = s_dma_rx_buf[slot];
    uart_dev->dma_rx_size      = UART_SIZE;
    uart_dev->dma_rx_processed = 0U;
    uart_dev->frame_buf        = s_frame_buf[slot];
    uart_dev->frame_buf_size   = UART_SIZE;
    uart_dev->dma_tx_buf       = s_dma_tx_buf[slot];
    uart_dev->tx_frame_len     = 0U;
    uart_dev->tx_busy          = false;
    uart_dev->initialized      = true;
    Func_Uart_FrameReset(uart_dev);

    /* 注册后 HAL 回调才能通过 huart 找到该实例 */
    s_uart_map[slot] = uart_dev;

    Func_Uart_ExitCritical(primask);

    /*
     * 启动 DMA 空闲接收。这一步会调用 HAL，放在临界区外，
     * 因为 HAL 内部可能等待 DMA 状态，关中断等待会有风险。
     * 若启动失败则回滚注册，不留下“已注册但收不到数据”的实例。
     */
    if (!Func_Uart_StartRx(uart_dev)) {
        primask = Func_Uart_EnterCritical();
        s_uart_map[slot]      = NULL;
        uart_dev->initialized = false;
        Func_Uart_ExitCritical(primask);
        return false;
    }

    return true;
}

void Func_Uart_DeInit(func_uart_base_t *uart_dev)
{
    if ((uart_dev == NULL) || !uart_dev->initialized) return;

    /* 先停硬件（临界区外，避免 HAL 内部等待时被关中断影响） */
    (void)HAL_UART_AbortReceive(uart_dev->huart);
    if (uart_dev->tx_busy) {
        (void)HAL_UART_AbortTransmit(uart_dev->huart);
    }

    /* 再从注册表移除并清理软件状态 */
    uint32_t primask = Func_Uart_EnterCritical();

    for (uint32_t i = 0U; i < UART_NUM; i++) {
        if (s_uart_map[i] == uart_dev) {
            s_uart_map[i] = NULL;
            break;
        }
    }

    uart_dev->tx_busy      = false;
    uart_dev->tx_frame_len = 0U;
    uart_dev->initialized  = false;
    Func_Uart_FrameReset(uart_dev);

    Func_Uart_ExitCritical(primask);
}

bool Func_Uart_Transmit(func_uart_base_t *uart_dev, const uint8_t *frame, uint16_t length)
{
    if ((uart_dev == NULL) || !uart_dev->initialized || (frame == NULL)) {
        return false;
    }

    /* ---- 校验：只接受完整帧 ---- */

    /* 长度必须能容纳包头 + 包尾，且不超过静态中转数组容量 */
    if ((length < 2U) || (length > UART_SIZE)) return false;

    /* 首字节必须是包头，末字节必须是包尾 */
    if (frame[0] != uart_dev->frame_head) return false;
    if (frame[length - 1U] != uart_dev->frame_tail) return false;

    /*
     * payload 中间出现包尾会导致发送端“按包尾定帧”时提前截断：
     * 例如 AA 01 55 02 55 会被当作 AA 01 55，剩下的 02 55 错位。
     * 因此直接拒绝这种帧，由调用者自行转义或修改协议。
     */
    for (uint16_t i = 1U; (uint16_t)(i + 1U) < length; i++) {
        if (frame[i] == uart_dev->frame_tail) return false;
    }

    bool queued = false;

    uint32_t primask = Func_Uart_EnterCritical();

    /*
     * 整帧入队：先检查剩余空间能否容纳“整帧”。
     * 不够就整帧丢弃，不写入任何字节——避免缓冲区出现半帧，
     * 也符合“空间不够就放弃新数据”的要求。
     */
    if (Circle_Buffer_Free(&uart_dev->txbuffer) >= length) {
        (void)Circle_Buffer_Put_Buffer(&uart_dev->txbuffer, frame, length);
        queued = true;
    } else {
        uart_dev->tx_drop_cnt++;
    }

    Func_Uart_ExitCritical(primask);

    /* 入队成功则尝试启动发送；若上一帧未发完，本帧会排队等待 */
    if (queued) {
        (void)Func_Uart_TxStart(uart_dev);
    }
    return queued;
}

bool Func_Uart_Receive(func_uart_base_t *uart_dev, uint8_t *frame, uint16_t length, uint16_t *out_length)
{
    if ((uart_dev == NULL) || !uart_dev->initialized ||
        (frame == NULL) || (length == 0U)) {
        return false;
    }
    if (out_length != NULL) *out_length = 0U;

    bool ok = false;
    uint16_t frame_len = 0U;

    uint32_t primask = Func_Uart_EnterCritical();

    /*
     * 先 Peek 扫描确定队首帧长度，再决定是否取出：
     *   - 帧完整且能放进用户缓冲区 -> Get_Buffer 搬走整帧；
     *   - 放不下 -> 直接返回 false，不消费任何字节。
     * 这样调用者不会拿到“被截断的半帧”，下次提供更大缓冲区时可以重试。
     */
    frame_len = Func_Uart_FindRxFrame(uart_dev);
    if ((frame_len >= 2U) && (frame_len <= length)) {
        if (Circle_Buffer_Get_Buffer(&uart_dev->rxbuffer, frame, frame_len) == frame_len) {
            ok = true;
        }
    }

    Func_Uart_ExitCritical(primask);

    if (ok && (out_length != NULL)) *out_length = frame_len;
    return ok;
}

/* ==================== HAL 回调函数 ==================== */

/*
 * DMA 接收事件回调（IDLE / TC / HT）。
 *
 * 参数 Size：本轮 DMA 启动以来累计收到的字节数，不是“本次新增字节数”。
 *           因此必须减去 dma_rx_processed，只解析新增部分。
 *
 * HT 事件：
 *   DMA 只写满一半，接收并未结束，不能重启 DMA（HAL 此时 RxState 仍为 BUSY）。
 *   解析完新增数据后直接返回，后半段会在 IDLE/TC 事件中处理。
 *
 * IDLE / TC 事件：
 *   本轮接收结束。HAL 已在回调前把 RxState 恢复为 READY，
 *   因此可以安全地调用 Func_Uart_StartRx() 重新启动下一轮 DMA。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    func_uart_base_t *dev = Func_Uart_Find(huart);
    if (dev == NULL) return;

    /* 防御：Size 不应超过本轮 DMA 缓冲长度 */
    if (Size > dev->dma_rx_size) Size = dev->dma_rx_size;

    /* 只处理 [dma_rx_processed, Size) 这段新增字节 */
    if (Size > dev->dma_rx_processed) {
        Func_Uart_RxParse(dev,
                          &dev->dma_rx_buf[dev->dma_rx_processed],
                          (uint16_t)(Size - dev->dma_rx_processed));
        dev->dma_rx_processed = (uint16_t)Size;
    }

    /* 半满事件不结束本轮接收，等待 IDLE / TC */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    /* IDLE / TC：本轮结束，重启 DMA 等待下一轮 */
    (void)Func_Uart_StartRx(dev);
}

/*
 * 发送完成回调。
 *
 * 触发时机：DMA 已把整帧数据搬完，并且最后一个字节的停止位已从引脚移出
 *          （USART TC 标志置位后 HAL 调用本回调）。
 *
 * 处理顺序：
 *   1. Drop 掉队首的 tx_frame_len 字节 —— 这一帧已经真正发完，可以释放空间；
 *   2. 清 tx_busy；
 *   3. 若队列里还有完整帧，取下一帧启动 DMA（同一临界区内完成，防止主循环插队
 *      造成状态竞争）。
 *
 * 注意：绝不能在 DMA 还没完成时提前 Drop，否则新数据可能覆盖正在发送的内存。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    func_uart_base_t *dev = Func_Uart_Find(huart);
    if (dev == NULL) return;

    uint32_t primask = Func_Uart_EnterCritical();

    if (dev->tx_frame_len > 0U) {
        Circle_Buffer_Drop(&dev->txbuffer, dev->tx_frame_len);
        dev->tx_frame_len = 0U;
    }
    dev->tx_busy = false;

    /* 直接在同一临界区内取下一帧，避免主循环插入造成额外中断嵌套 */
    (void)Func_Uart_TxStartLocked(dev);

    Func_Uart_ExitCritical(primask);
}

/*
 * 串口错误回调（溢出错误、DMA 错误等）。
 *
 * HAL 在 DMA 模式下遇到错误会中止本次收发，回调前已把状态恢复为 READY。
 * 这里的恢复策略：
 *   1. 丢弃当前正在发送的帧（它可能只发出去一部分，继续发会错位）；
 *   2. 复位组帧状态机（当前半帧作废，重新找包头）；
 *   3. 中止并重启接收，同时尝试发送队列中的下一帧。
 *
 * abort/启动 DMA 都放在临界区外，避免关中断状态下等待相关标志。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    func_uart_base_t *dev = Func_Uart_Find(huart);
    if (dev == NULL) return;

    uint32_t primask = Func_Uart_EnterCritical();

    dev->err_cnt++;

    /* 发送被打断：丢弃当前帧（可能只发出去一部分），避免指针错位 */
    uint16_t pending = dev->tx_frame_len;
    dev->tx_frame_len = 0U;
    dev->tx_busy      = false;
    if (pending > 0U) {
        Circle_Buffer_Drop(&dev->txbuffer, pending);
    }

    /* 接收组帧状态作废，重新同步包头 */
    Func_Uart_FrameReset(dev);

    Func_Uart_ExitCritical(primask);

    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_AbortTransmit(huart);
    (void)Func_Uart_StartRx(dev);
    (void)Func_Uart_TxStart(dev);
}
