# func_uart_0 优化历史

> 仓库：`/home/llsjjz/STM32CubeMX/project/stm32f103zet6_uart`（分支 `main`）
>
> Git 提交：
> - `23053c1 first commit`（2026-11-22 14:04）
> - `e6880fb 1`（14:08，仅新增 `.gitignore`）
> - `8fefee5 0版完成`（16:38，最终版）
>
> 说明：git 中 `func_uart_0` 只有“初版”和“最终版”两个快照；中间的多次工作区迭代没有单独提交。本文按时间线性叙述，并用会话期间保存的版本快照（`/tmp/uart_step1_bak/`）与逐次 diff 还原每一次修改。

---

## 一、背景

2026-11-21 的原始需求是优化串口模块 `func_uart`：

- 每个串口两个环形缓冲区，分别用于发送和接收；
- 回调函数中使用静态数组做中转；
- 考虑包头和包尾，空间不够时放弃新数据；
- 最终只保留四个接口：初始化、注销、发送、接收；
- 发送接口只接受含包头和包尾的完整帧，一次 DMA 只发一帧；接收端每次只取出一帧。

`func_uart.c` 按“包头 + payload + 包尾”实现了上述方案。`func_uart_0.c` 是另一条实现路线：不再扫描包头包尾，而是为每个方向额外维护一个**长度队列**，每帧在长度队列中占 2 字节（小端），数据队列保存帧内容。两条路线功能重叠，因此一直存在同名符号冲突。

本次任务的目标：在保留“长度队列 + 环形缓冲区”思路的前提下，把 `func_uart_0` 从原型改造成可实际收发的实现。

---

## 二、版本总览

| 阶段 | 时间 | 完成后行数 | 说明 |
|---|---|---:|---|
| v0 原型 | 2026-11-22 13:56 / 提交 `23053c1`（14:04） | 98 | 只有入队逻辑，无回调；git 中为整文件注释状态 |
| v1 引入回调 | 15:16 | 121 | 增加静态 RX 中转、启动函数、Rx/Tx 回调 |
| v2 用户修正 | 15:40 | 172 | 修正回调匹配、队列参数、Init 方向，改名 `s_rxbuffer` |
| 步骤 1 | 15:50 | 174 | 构建基线、空指针与回滚 |
| 步骤 2 | 15:53 | 190 | RX 通路（IDLE 分帧） |
| 步骤 3 | 16:00 | 267 | TX 通路；随后用户重构（改名 BaseRx/BaseTx、整理注释），行为不变 |
| 步骤 4 | 16:21 | 415 | 临界区、Get/Put 重写、DeInit、错误恢复 |
| 步骤 5 | 16:31 | 458 | 文件头原理注释、最终干净编译 |

> 行数来自各阶段文件快照。v2 之后 `.c` 的签名已全部使用 `uint16_t`，头文件在 14:52 先行完成了 `uint16_t` 与 `UART_MAX/UART_MAXNUM` 的改名。

---

## 三、v0：长度队列原型（98 行）

> Git `23053c1` 中该文件为整文件注释状态；工作区中曾以未注释的同一份代码作为优化起点（会话读取到 98 行未注释版本，逻辑与注释版一致）。

### 3.1 结构

```c
static func_uart_base_t *s_uart_map[UART_NUM] = {NULL};
static uint8_t s_txbuffer_num[UART_NUM][UART_SIZE];
static uint8_t s_rxbuffer_num[UART_NUM][UART_SIZE];
```

每个实例持有：

- `rxbuffer` / `txbuffer`：用户提供的数据环形缓冲区；
- `rxbuffer_num` / `txbuffer_num`：长度环形缓冲区；
- `huart`：串口句柄。

### 3.2 行为

- `Func_Uart_Init()`：绑定用户缓冲区，注册到 `s_uart_map`，初始化两个长度队列，然后
  `HAL_UARTEx_ReceiveToIdle_DMA(huart, dev->rxbuffer.address, rx_length)`，关闭 HT 中断。
- `Func_Uart_Transmit()`：检查长度队列未满、数据队列空间足够，把数据写入 `txbuffer`，把 1 字节长度写入 `txbuffer_num`。**没有启动 DMA。**
- `Func_Uart_Receive()`：从长度队列 `Peek` 1 字节长度，判断是否超过调用者容量，然后 `Get` 掉长度，再按**调用者容量**而不是帧长调用 `Get_Buffer`。

### 3.3 问题与原因

1. **接收没有生产者**
   文件里没有 `HAL_UARTEx_RxEventCallback`，`rxbuffer_num` 永远为空，`Func_Uart_Receive` 恒返回 `false`。

2. **DMA 直接写用户环形缓冲区的底层数组**
   DMA 只改数组内容，不更新 `head/tail/count`；而且 DMA 从数组起始位置覆盖写，会破坏尚未读取的旧数据。

3. **DMA 不会重启**
   `ReceiveToIdle_DMA` 在 IDLE/TC 后停止；没有回调重启，第一次接收结束后串口不再收数。

4. **发送只入队，不发送**
   没有 `HAL_UART_Transmit_DMA`，也没有 `TxCplt` 续发；队列只进不出。

5. **接收长度用错**
   ```c
   date = Circle_Buffer_Get(&dev->rxbuffer_num, 0);
   Circle_Buffer_Get_Buffer(&dev->rxbuffer, frame, length);  // 应为 date
   ```
   取出的是调用者容量，不是这一帧的实际长度；`Get_Buffer` 的返回值也没有校验。

6. **空队列仍返回 true**
   长度队列为空时，`Peek` 失败后代码仍继续 `Get` 并返回 `true`。

7. **长度只能存 1 字节**
   当时 `Put(txbuffer_num, length)` 只写 `uint8_t`，受 `UART_SIZE = 255` 限制。

8. **初始化失败不回滚**
   `HAL_UARTEx_ReceiveToIdle_DMA` 失败直接 `return false`，`s_uart_map` 槽位仍被占用；`hdmarx` 也没有判空。

9. **`.c` 与 `.h` 类型不一致**
   头文件已改为 `uint16_t` 长度，`.c` 仍是 `uint8_t`，编译时类型冲突。

10. **与 `func_uart` 冲突**
    两个头文件共用 `FUNC_UART_H` 守卫，两个 `.c` 定义同名 `Func_Uart_*` 与 HAL 回调，CMake 又同时编译两者。

---

## 四、v1：引入回调与静态中转（121 行，15:16）

### 4.1 改动

- 新增静态接收落点：
  ```c
  static uint8_t rxbuffer[UART_NUM][UART_MAX];
  ```
- 新增 `Func_Uart_ReRxStart()` / `Func_Uart_ReTxStart()`，集中封装 HAL 启动；
- 新增 `HAL_UARTEx_RxEventCallback()` 与 `HAL_UART_TxCpltCallback()`；
- 长度队列改为 2 字节小端；
- `Func_Uart_Get()` 增加了 `Peek` 失败时的 `else return false`；
- 头文件在 14:52 已先行改为 `uint16_t` 长度、`UART_MAX/UART_MAXNUM`。

### 4.2 新引入的问题

1. **回调在空槽位上解引用空指针**
   ```c
   for (...) {
       if (s_uart_map[i] == NULL) {
           s_uart_map[i]->huart = huart;   // NULL 解引用
           registered = true;
           break;
       }
   }
   ```
   任何串口中断都会 HardFault；而且找到的是空槽位，不是发出中断的实例。

2. **`Init` 对启动结果判断反向**
   ```c
   if (Func_Uart_ReRxStart(huart, rxbuffer)) return false;
   ```
   启动成功反而返回失败，启动失败反而继续返回成功。

3. **形参与静态数组同名遮蔽**
   `Init` 的形参 `rxbuffer` 遮蔽了文件级静态数组，第一轮 DMA 实际写入用户环形缓冲区；而回调却从静态 `rxbuffer[i]` 取数据，前后不一致。

4. **RX 回调把数据队列当长度队列**
   ```c
   Func_Uart_Put(&dev->rxbuffer, &dev->rxbuffer, data, Size);
   ```
   两个参数是同一个队列，长度与数据混写。

5. **TX 完成回调使用 2 字节栈数组发送 255 字节**
   ```c
   uint8_t txbuffer[UART_NUM];                   // UART_NUM == 2
   Func_Uart_ReTxStart(huart, txbuffer, UART_MAX); // 发送 255 字节
   ```
   越界读取栈内存；局部数组在回调返回后失效，DMA 源地址不可靠。

6. **发送仍不启动**
   `Func_Uart_Transmit()` 依旧只入队；`ReTxStart` 只在 `TxCplt` 中调用。

7. **其它**
   `hdmarx` 未判空；初始化失败不回滚；回调中有空 `if` 块和冗余的 `huart` 自赋值。

---

## 五、v2：用户自行修正（172 行，15:40）

这一版由用户直接修改，主要修复了 v1 中最危险的问题：

1. 静态数组改名 `s_rxbuffer`，消除与 `Init` 形参的同名遮蔽；
2. `Init` 改为 `if (!Func_Uart_ReRxStart(huart, s_rxbuffer[i])) return false;`，判断方向正确，DMA 落点统一为静态数组；
3. 两个回调改为按 `s_uart_map[i] != NULL && s_uart_map[i]->huart == huart` 查找实例，消除空指针解引用；
4. RX 回调的 `Func_Uart_Put` 参数修正为 `&rxbuffer, &rxbuffer_num`；
5. TX 回调改为 `uint8_t txbuffer[UART_MAX]`，并在 `Func_Uart_Get` 失败时不再启动发送。

仍存在的问题：

- 回调中仍有空 `if` 块与冗余 `s_uart_map[i]->huart = huart`；
- `hdmarx` 未判空；`Init` 失败不回滚；
- `TxCplt` 仍使用栈数组，DMA 源生命周期不安全，且固定发送 `UART_MAX`；
- `Transmit` 仍不启动发送；
- `Get` 先 Drop 长度再取数据、不校验返回值；
- 没有临界区、`DeInit`、错误回调、`initialized` 检查。

---

## 六、步骤 1：构建基线与崩溃修复（完成后 174 行，15:50）

**目标：先建立“不崩、可编译”的基线。**

### 6.1 构建层面

- `func/CMakeLists.txt` 注释掉 `func_uart.c`：
  ```cmake
  # ${CMAKE_CURRENT_SOURCE_DIR}/uart/func_uart.c   # 与 func_uart_0.c 同名符号冲突，暂不参与构建
  ```
  原因：两个文件定义同名 `Func_Uart_Init/Transmit/Receive` 以及同名 HAL 弱回调，静态库中会出现两套符号，链接结果取决于归档成员顺序。
- `func_uart_0.h` 守卫改为 `FUNC_UART_0_H`。
  原因：原守卫与 `func_uart.h` 相同，若同一编译单元同时包含两个头文件，第二个会被静默跳过，导致结构体定义与宏不一致。

### 6.2 运行安全

- `BaseRx`（当时名为 `ReRxStart`）中给 `__HAL_DMA_DISABLE_IT` 增加 `huart->hdmarx != NULL` 判断；
- `Init` 启动失败时：
  ```c
  if (!Func_Uart_ReRxStart(huart, s_rxbuffer[i])) {
      s_uart_map[i] = NULL;   // 回滚注册，避免槽位泄漏
      return false;
  }
  ```
- 删除两个回调中的空 `if` 块与冗余的 `huart` 自赋值。

**验证**：编译通过；本步不做功能验证，RX/TX 尚未打通。

---

## 七、步骤 2：打通 RX 通路（完成后 190 行，15:53）

### 7.1 结构体新增状态

```c
uint8_t slot;          /* 静态中转数组槽位 */
uint32_t rx_drop_cnt;  /* 接收整帧丢弃计数 */
```

`Init` 中记录 `dev->slot = i` 并清零计数。

### 7.2 重写 RX 回调

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. 按 huart 找到实例
    // 2. Size 截断到 UART_MAX
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;                       // HT 不是帧边界
    }
    if (Size > 0U) {
        if (!Func_Uart_Put(&dev->rxbuffer, &dev->rxbuffer_num,
                           s_rxbuffer[dev->slot], Size)) {
            dev->rx_drop_cnt++;       // 任一队列空间不足：整帧丢弃
        }
    }
    (void)Func_Uart_BaseRx(huart, s_rxbuffer[dev->slot]);  // 重启 DMA
}
```

### 7.3 为什么不做 `[processed, Size)` 增量处理

计划中曾考虑 `dma_rx_processed` 偏移，但本模块确认采用 **IDLE/TC 边界即一帧**：

- HT 事件发生在帧中间，不能把前半段当作一帧提交；
- IDLE/TC 时 HAL 给出的 `Size` 是本轮累计总长度，缓冲区内容仍是 `[0, Size)`；
- 因此正确处理是：**忽略 HT，IDLE/TC 时把整段 `[0, Size)` 作为一帧提交**。

**验证**：编译通过。

---

## 八、步骤 3：打通 TX 通路（完成后 267 行，16:00）

### 8.1 新增静态发送中转与状态

```c
static uint8_t s_txbuffer[UART_NUM][UART_MAX];

bool tx_busy;          /* DMA 正在发送一帧 */
uint16_t tx_frame_len; /* 当前发送帧长，完成前不释放 */
```

### 8.2 新增辅助函数

- `Func_Uart_PeekLength()`：从长度队列窥视 2 字节小端帧长，不出队；
- `Func_Uart_CopyOut()`：把数据队列队首一帧按“环形回绕”拆成两段 `memcpy`，拷到连续静态数组；
- `Func_Uart_StartTx()`：
  1. `tx_busy` 为真则直接返回；
  2. 窥视帧长，校验 `1 <= len <= UART_MAX`；
  3. 校验数据队列中确有 `len` 字节；
  4. 拷入 `s_txbuffer[slot]`；
  5. 调用 `HAL_UART_Transmit_DMA()`；
  6. 记录 `tx_frame_len`、置 `tx_busy`。

### 8.3 `Transmit` 与 `TxCplt`

```c
bool Func_Uart_Transmit(...)
{
    // 校验长度/空间
    if (!Func_Uart_Put(&dev->txbuffer, &dev->txbuffer_num, frame, length)) return false;
    (void)Func_Uart_StartTx(dev);   // 空闲时立即发送，忙时排队
    return true;
}
```

```c
void HAL_UART_TxCpltCallback(...)
{
    frame_len = dev->tx_frame_len;
    dev->tx_frame_len = 0;
    dev->tx_busy = false;
    if (frame_len > 0) {
        Circle_Buffer_Drop(&dev->txbuffer, frame_len);   // 按实际帧长释放
        Circle_Buffer_Drop(&dev->txbuffer_num, 2);
    }
    (void)Func_Uart_StartTx(dev);                        // 续发下一帧
}
```

### 8.4 关键点

- **DMA 源必须是静态内存**：栈数组在函数返回后失效，且 DMA 传输期间不能被覆盖；
- **发送完成前不 Drop**：只有 `TxCplt` 才释放队列中的帧，保证 DMA 读取期间数据有效；
- **长度使用实际值**，不再固定 `UART_MAX`；
- `Func_Uart_Get()` 中重复的长度解析改为调用 `Func_Uart_PeekLength()`。

**验证**：编译通过。

---

## 九、用户重构（16:18）

在步骤 3 与步骤 4 之间，用户对 `.c` 做了一次不改变行为的整理：

- `Func_Uart_ReRxStart` → `Func_Uart_BaseRx`；
- `Func_Uart_ReTxStart` → `Func_Uart_BaseTx`；
- 增加“基础发送与接收 / 私有 API / 公共 API / 中断回调”分区注释；
- 调整函数与静态数组的排列顺序。

步骤 3 的功能改动全部保留。

---

## 十、步骤 4：并发安全与健壮性（完成后 415 行，16:21）

### 10.1 临界区

队列同时被主循环（`Transmit/Receive`）与串口中断（`RxEvent/TxCplt`）访问。新增：

```c
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
```

保存并恢复 PRIMASK，嵌套调用不会破坏中断状态。

### 10.2 重写 `Func_Uart_Get`

```c
static bool Func_Uart_Get(..., uint16_t capacity, uint16_t *out_length)
{
    if (out_length) *out_length = 0;
    primask = Func_Uart_EnterCritical();

    if (Func_Uart_PeekLength(buffer_num, &need_length)) {
        if (need_length == 0U) {
            Circle_Buffer_Drop(buffer_num, 2U);          // 非法长度项
        } else if (need_length <= capacity &&
                   Circle_Buffer_Used(buffer) >= need_length) {
            if (Circle_Buffer_Get_Buffer(buffer, frame, need_length) == need_length) {
                Circle_Buffer_Drop(buffer_num, 2U);      // 成功后才释放长度
                *out_length = need_length;
                ok = true;
            }
        }
    }
    Func_Uart_ExitCritical(primask);
    return ok;
}
```

解决的问题：

- 原实现先 Drop 长度再取数据，数据不足时会永久错位；
- 不校验 `Get_Buffer` 返回值，可能返回“成功但 0 字节”；
- `need_length == 0` 会占住队首；
- 调用者无法知道实际帧长。

### 10.3 重写 `Func_Uart_Put`

```c
primask = Func_Uart_EnterCritical();
if (Circle_Buffer_Free(buffer_num) >= 2U &&
    Circle_Buffer_Free(buffer) >= length) {
    Circle_Buffer_Put_Buffer(buffer, frame, length);       // 先写数据
    Circle_Buffer_Put_Buffer(buffer_num, need_length, 2U); // 后写长度
    ok = true;
}
Func_Uart_ExitCritical(primask);
```

- 两边空间在临界区内一次性确认，`Put_Buffer` 不会只写一半；
- 先数据后长度：任何时刻看到长度项，数据一定已经就绪；
- 原来先写长度，若数据写入失败会留下“有长度、无数据”的记录。

### 10.4 `StartTx` 与 `TxCplt` 的并发保护

- `StartTx` 在临界区内检查 `tx_busy`、窥视长度、校验数据量、拷贝帧，并**先置 `tx_busy = true` 再退出临界区**，然后调用 HAL 启动 DMA。这样主循环与 `TxCplt` 不可能同时启动两帧；HAL 启动失败时再进入临界区清除忙状态。
- `TxCplt` 的“读帧长 → 清忙 → Drop 两个队列”整体放入临界区，之后调用 `StartTx` 续发。

### 10.5 接口与生命周期

- `Func_Uart_Transmit` / `Func_Uart_Receive` 增加 `dev == NULL`、`!dev->initialized` 检查；
- `Func_Uart_Receive` 增加 `uint16_t *out_length`；
- 新增 `bool initialized`、`uint32_t err_cnt`；
- 新增 `Func_Uart_DeInit()`：中止收发、清除注册表、复位忙状态；
- 新增 `HAL_UART_ErrorCallback()`：丢弃在途发送帧、中止并重启收发；
- `Init` 启动失败时同时清除 `initialized` 并回滚槽位；
- 禁止同一个 `dev` 重复初始化、同一个 `huart` 重复注册。

**验证**：编译通过。

---

## 十一、步骤 5：注释与最终验证（458 行，16:31）

- 在 `func_uart_0.c` 顶部增加完整原理注释：
  - IDLE/TC 边界即一帧；
  - 静态中转数组的必要性；
  - 长度队列与数据队列的配合；
  - PRIMASK 临界区；
  - 单帧 255 字节上限与“帧内停顿会被拆帧”的限制。
- 执行干净完整编译（`--clean-first`）：
  - **0 error，0 warning**；
  - RAM 1792 B / 64 KB（2.73%），FLASH 10064 B / 512 KB（1.92%）。

> 此时 `main.c` 尚未调用该模块，链接器通过 `--gc-sections` 回收了未引用段，因此资源占用数字与之前相同。实际引用后预计增加约 2 KB RAM（4 块 `[UART_NUM][UART_MAX]` 静态数组）及少量 Flash。

---

## 十二、最终集成（提交 `8fefee5 0版完成`，16:38）

最终提交包含三处集成改动：

1. `Core/Src/main.c`：
   ```c
   - #include "func_uart.h"
   + #include "func_uart_0.h"
   ```
2. `func/CMakeLists.txt`：`func_uart.c` 被注释，只编译 `circle_buffer.c` 与 `func_uart_0.c`；
3. `func_uart_0.h`：最终 45 行，包含：
   - `UART_NUM 2`、`UART_MAX 255`、`UART_MAXNUM 255`；
   - 数据/长度队列；
   - 内部状态 `slot`、`rx_drop_cnt`、`tx_busy`、`tx_frame_len`、`err_cnt`、`initialized`；
   - `Init / DeInit / Transmit / Receive` 四个公共接口。

最终数据流：

```text
接收：
UART RX -> DMA -> s_rxbuffer[slot]
        -> IDLE/TC 回调把整段 [0, Size) 作为一帧
        -> rxbuffer + rxbuffer_num
        -> Func_Uart_Receive() 一次取一帧

发送：
Func_Uart_Transmit()
        -> rxbuffer/txbuffer_num 整帧入队
        -> Func_Uart_StartTx() 窥视帧长、拷入 s_txbuffer[slot]
        -> HAL_UART_Transmit_DMA() 一帧一次 DMA
        -> TxCplt 按实际帧长释放两个队列并续发
```

---

## 十三、问题与修复对照表

| # | 问题 | 出现版本 | 原因 | 修复 |
|---:|---|---|---|---|
| 1 | 回调空槽位解引用 NULL | v1 | 查找条件写成 `== NULL` | 步骤 1：改为非空且 `huart` 匹配 |
| 2 | Init 启动结果判断反向 | v1 | `if (start(...)) return false` | 用户 v2 修正；步骤 1 补回滚 |
| 3 | 同名符号 / 同名守卫冲突 | v0-v2 | 与 `func_uart` 并行编译 | 步骤 1：CMake 排除 + 守卫改名 |
| 4 | `hdmarx` 空指针 | v1 | 未判空 | 步骤 1 |
| 5 | 初始化失败槽位泄漏 | v0-v2 | 先注册后启动，失败不清理 | 步骤 1 / 步骤 4 |
| 6 | DMA 写用户环形缓冲区 | v0 | 直接把 `rxbuffer.address` 交给 DMA | v1/v2 改为静态 `s_rxbuffer` |
| 7 | 回调提交参数错误 | v1 | 数据队列当长度队列 | v2 修正 |
| 8 | HT/IDLE/TC 未区分 | v1-v2 | 所有事件同样处理 | 步骤 2：忽略 HT，IDLE/TC 整帧提交 |
| 9 | 接收丢帧无统计 | v1-v2 | 无计数 | 步骤 2：`rx_drop_cnt` |
| 10 | 发送从不启动 DMA | v0-v2 | `Transmit` 只入队 | 步骤 3：`StartTx` + `Transmit` 启动 |
| 11 | 2 字节栈数组发送 255 字节 | v1 | 局部缓冲区过小 | v2 改为 255 字节；步骤 3 改为静态数组 |
| 12 | 固定发送 255 字节 | v1-v2 | 不读取实际帧长 | 步骤 3：按长度队列实际值发送 |
| 13 | 发送完成前释放帧 | v0-v2 | 无 TxCplt 管理 | 步骤 3：TxCplt 才 Drop |
| 14 | Get 先 Drop 后取数 | v0-v2 | 顺序错误 | 步骤 4：成功取数后才 Drop |
| 15 | 无临界区 | v0-v3 | 主循环与中断并发 | 步骤 4：PRIMASK 临界区 |
| 16 | `need_length == 0` 卡队列 | v0-v2 | 未处理非法长度 | 步骤 4：丢弃该项 |
| 17 | 调用者不知道帧长 | v0-v4 | 接口无输出参数 | 步骤 4：`out_length` |
| 18 | 无注销 / 错误恢复 | v0-v4 | 功能缺失 | 步骤 4：`DeInit` + `ErrorCallback` |
| 19 | 重复初始化 | v0-v4 | 只检查 huart，不检查 dev | 步骤 4 |

---

## 十四、验证与限制

### 14.1 已验证

- 每个步骤均执行 `cmake --build build/Debug -j$(nproc)`，编译通过；
- 步骤 5 干净完整编译：0 error、0 warning；
- 最终资源占用：RAM 1792 B（2.73%），FLASH 10064 B（1.92%）。

### 14.2 待硬件验证

1. TX-RX 回环，发送 5 字节，`Receive` 取回相同数据且 `out_length == 5`；
2. 两帧之间间隔大于 1 个字符时间，接收端取出两帧；
3. 填满接收缓冲区后再收，新帧整帧丢弃、`rx_drop_cnt` 增加、旧帧不受影响；
4. 连续发送 100 帧，顺序正确、无丢帧；
5. 制造溢出/DMA 错误，确认 `HAL_UART_ErrorCallback` 后收发恢复。

### 14.3 已知限制

- 单帧最大 `UART_MAX = 255` 字节；
- 采用 IDLE/TC 边界分帧，发送方在帧内停顿会被拆成两帧；
- 超过 255 字节的帧会被 TC 事件拆段；
- `Init` 按“主循环、串口中断启动前调用”设计；
- 若 `StartTx` 第一次启动 DMA 失败，帧保留在队列中，需要下一次 `Transmit` 或错误回调触发重试。

---

## 附录：中间版本快照

| 快照 | 说明 |
|---|---|
| `/tmp/uart_step1_bak/func_uart_0.c` | v2（15:40） |
| `/tmp/uart_step1_bak/func_uart_0.c.1540` | v2（15:40） |
| `/tmp/uart_step1_bak/func_uart_0.c.step2` | 步骤 1 后 |
| `/tmp/uart_step1_bak/func_uart_0.c.step3` | 步骤 2 后 |
| `/tmp/uart_step1_bak/func_uart_0.c.step4` | 步骤 3 + 用户重构后 |
| `/tmp/uart_step1_bak/func_uart_0.c.step5` | 步骤 4 后 |
| 当前工作区 / `8fefee5` | 步骤 5 后最终版 |

> `/tmp` 为临时目录，重启后可能被清理；如需长期保留，建议将关键快照放入仓库的 `docs/` 或 `history/` 目录。
