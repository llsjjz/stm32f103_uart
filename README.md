# stm32f103zet6_uart

基于 **STM32F103ZET6** + **STM32 HAL** 的串口模块工程：用 **DMA + IDLE 中断** 收发，配合**环形缓冲区**与**长度队列**实现「一帧一帧」的整帧收发，不丢半帧、不覆盖未读数据。

工程由 STM32CubeMX 生成骨架，构建系统为 **CMake + Ninja + arm-none-eabi-gcc**，可用 **OpenOCD + ST-Link** 烧录与调试。

---

## 特性

- **整帧收发**：每次 DMA 传输 / 每次 `Func_Uart_Receive` 都对应一个完整帧，不会把半帧交给用户。
- **长度队列**：不依赖包头包尾，每帧在独立的长度队列中占 2 字节（小端），帧长和数据分开存放。
- **环形缓冲区**：收发各一个，容量可自定义，写满时整帧丢弃而不是覆盖未读数据。
- **静态中转数组**：DMA 缓冲区放在 `.bss`，地址固定、生命周期贯穿运行期，中断里不分配大栈。
- **中断安全**：所有「检查 + 修改」操作都在 `PRIMASK` 临界区内完成，主循环与中断可并发调用。
- **发送排队**：发送忙时帧自动排队，`TxCplt` 后自动续发下一帧；DMA 读数据期间不释放队列。
- **错误恢复**：`HAL_UART_ErrorCallback` 中丢弃被中断的帧、重启收发，避免后续帧错位。
- **多实例**：最多同时管理 `UART_NUM`（默认 2）个串口，互不干扰。

## 硬件资源

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F103ZET6（Cortex-M3，512KB Flash / 64KB RAM） |
| 系统时钟 | HSI 8 MHz（未启用 PLL） |
| 串口 | USART1，115200 8-N-1，无硬件流控 |
| 引脚 | PA9 = USART1_TX，PA10 = USART1_RX |
| DMA | DMA1 Channel4（TX，MEM→PERIPH）、DMA1 Channel5（RX，PERIPH→MEM） |
| 中断 | USART1_IRQn（优先级 1）、DMA1_Channel4/5_IRQn（优先级 0） |

## 目录结构

```
.
├── Core/
│   ├── Inc/                  # CubeMX 生成的 HAL 配置头文件
│   └── Src/
│       ├── main.c            # 应用入口：初始化 + 回环演示
│       ├── usart.c           # USART1 + DMA 初始化
│       ├── dma.c             # DMA 时钟与中断配置
│       └── ...
├── Drivers/                  # STM32F1 HAL / CMSIS（CubeMX 生成，勿改）
├── func/                     # 用户模块（核心业务代码）
│   ├── CMakeLists.txt        # func_uart 静态库
│   └── uart/
│       ├── circle_buffer.c/.h    # 通用环形缓冲区
│       ├── func_uart_0.c/.h      # 【当前启用】长度队列 + IDLE 分帧方案
│       ├── func_uart.c/.h        # 【未参与构建】包头包尾方案
│       └── func_uart_0_优化历史.md  # v0 → 最终版的迭代记录
├── cmake/                    # 工具链文件与 CubeMX 生成的 CMake 片段
├── CMakeLists.txt            # 顶层构建脚本
├── CMakePresets.json         # Debug / Release 预设
├── flash.cfg                 # OpenOCD 烧录脚本
├── STM32F103xx_FLASH.ld      # 链接脚本
├── startup_stm32f103xe.s     # 启动文件
└── stm32f103zet6_uart.ioc    # CubeMX 工程文件
```

> `func/uart/func_uart.c` 与 `func_uart_0.c` 定义了同名的回调函数与宏，二者只能选其一参与构建。当前 `func/CMakeLists.txt` 中已注释掉前者，使用 `func_uart_0.c`。

## 软件架构

### 接收通路

```
UART RX → DMA → s_rxbuffer[slot]（静态中转）
   │
   │  IDLE / 传输完成中断
   ▼
HAL_UARTEx_RxEventCallback
   │  本轮收到的整段数据 = 一帧
   ▼
rxbuffer（数据环形队列） + rxbuffer_num（2 字节长度队列）
   │
   ▼
Func_Uart_Receive()  →  一次取出一帧
```

- **半满（HT）事件不是帧边界**，直接忽略，继续等待 IDLE / TC。
- 两个队列任一空间不足时**整帧丢弃**并累加 `rx_drop_cnt`，绝不写入半帧。

### 发送通路

```
Func_Uart_Transmit()
   │  校验长度与空间，整帧写入 txbuffer + txbuffer_num
   ▼
Func_Uart_StartTx()（空闲时启动）
   │  窥视帧长 → 拷入 s_txbuffer[slot] → HAL_UART_Transmit_DMA()
   ▼
HAL_UART_TxCpltCallback
   │  按实际帧长释放两个队列 → 续发下一帧
```

- 一次 DMA 只发一帧，发送期间不释放队列，保证 DMA 读取的数据不被覆盖。

## 快速开始

### 1. 依赖

- `arm-none-eabi-gcc`（需在 `PATH` 中）
- CMake ≥ 3.22
- Ninja
- OpenOCD（烧录 / 调试）、ST-Link 调试器

### 2. 配置与构建

```bash
# 配置（生成 build/Debug）
cmake --preset Debug

# 构建，产物在 build/Debug/：
#   stm32f103zet6_uart.elf
#   stm32f103zet6_uart.hex
cmake --build --preset Debug
```

Release 构建同理，把 `Debug` 换成 `Release`。

### 3. 烧录

```bash
openocd -f flash.cfg
```

`flash.cfg` 使用**绝对路径**指向 `build/Debug/stm32f103zet6_uart.elf`，换机器或换目录后需要同步修改该路径，例如：

```tcl
source [find interface/stlink.cfg]
source [find target/stm32f1x.cfg]

program /你的/项目/路径/build/Debug/stm32f103zet6_uart.elf verify reset exit
```

### 4. VS Code（可选）

仓库内的 `.vscode/tasks.json` 提供「构建 Debug」「清理重建」「OpenOCD 烧录」等任务，`launch.json` 提供 ST-Link 调试配置。该目录已被 `.gitignore` 忽略，属于本地配置，可自行调整。

## 使用示例

`main.c` 中的回环演示：收到一帧就原样发回。

```c
#include "func_uart_0.h"

uint8_t rxbuffer[255];
uint8_t txbuffer[255];
func_uart_base_t uart1;

Func_Uart_Init(&uart1, rxbuffer, 255, txbuffer, 255, &huart1);

while (1)
{
    char txt[255] = {0};
    uint16_t leng = 0;

    if (Func_Uart_Receive(&uart1, (uint8_t *)txt, 20, &leng))
    {
        Func_Uart_Transmit(&uart1, (uint8_t *)txt, leng);
    }

    HAL_Delay(100);
}
```

### API

| 函数 | 说明 |
| --- | --- |
| `Func_Uart_Init(dev, rxbuf, rxlen, txbuf, txlen, huart)` | 绑定缓冲区与句柄并启动 DMA 接收；返回 `false` 表示参数非法、重复注册或槽位已满 |
| `Func_Uart_DeInit(dev)` | 停止收发并注销实例 |
| `Func_Uart_Transmit(dev, frame, length)` | 整帧入队并（空闲时）立即发送；空间不足或长度非法返回 `false` |
| `Func_Uart_Receive(dev, frame, capacity, out_length)` | 取出一帧；无完整帧或帧长超过 `capacity` 返回 `false` |

> `UART_NUM`（最大实例数）、`UART_MAX`（单帧上限，默认 255）、`UART_MAXNUM`（长度队列容量）均在 `func_uart_0.h` 中定义。

## 帧格式与限制

- **不依赖包头/包尾**，帧边界由**总线空闲（IDLE）** 或 DMA 传输完成确定：一次连续收到的数据即是一帧。
- 因此**发送方不能在帧内出现明显停顿**（超过约 1 个字符时间），否则一帧会被拆成两帧。
- 单帧长度不得超过 `UART_MAX`（默认 255 字节）。如需更长帧，必须同时增大 `UART_MAX`、增加 `s_rxbuffer`/`s_txbuffer` 尺寸，RAM 占用会成倍上升。
- 发送队列满时整帧丢弃（`Func_Uart_Transmit` 返回 `false`），不会阻塞调用者。

## 设计要点

1. **环形缓冲用 `count` 区分空/满**：`head == tail` 在空和满时都成立，额外维护 `count` 才能判断；指针回绕由 `Put_Buffer` / `Get_Buffer` 内部拆成两次 `memcpy` 处理。
2. **先写数据、后写长度**：任何时刻只要能看到长度项，数据就一定已经就绪；取出时校验实际读出长度，成功后才丢弃长度项，避免两个队列永久错位。
3. **临界区保存/恢复 `PRIMASK`**：嵌套调用不会破坏原中断状态。
4. **静态中转数组按槽位分配**：DMA 需要地址固定、生命周期长的内存；中断服务函数也不能使用大局部数组。
5. **发送完成才释放队列**：`TxCplt` 中按实际帧长 `Drop`，保证 DMA 读期间数据有效。

## 性能参考（Debug 构建）

```
   text    data     bss     dec     hex
  15680      12    3836   19528    4c48
```

## 相关文档

- [`func/uart/func_uart_0_优化历史.md`](func/uart/func_uart_0_优化历史.md)：从 v0 原型到最终版的逐次修改记录与设计取舍。
- [`func/uart/circle_buffer.h`](func/uart/circle_buffer.h)：环形缓冲区原理与接口说明。
- [`func/uart/func_uart_0.c`](func/uart/func_uart_0.c)：模块数据流与并发规则的详细注释。

## 许可

工程骨架与 HAL 驱动版权归 STMicroelectronics 所有，遵循其随附许可条款；`func/` 下的用户代码可自行决定许可方式。