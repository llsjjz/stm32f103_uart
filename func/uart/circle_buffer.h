#ifndef __CIRCLE_BUFFER_H__
#define __CIRCLE_BUFFER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * ============================ 环形缓冲区原理 ============================
 *
 *   address ─┐
 *            ▼
 *   ┌────────┬──────────────────────────────────┬─────────┐
 *   │  空闲  │           已存数据               │  空闲   │
 *   └────────┴──────────────────────────────────┴─────────┘
 *            ▲                                  ▲
 *          head（读指针）                    tail（写指针）
 *
 * - head：下一次读（出队）的位置，始终指向缓冲区里最早的数据；
 * - tail：下一次写（入队）的位置，始终指向第一个空闲字节；
 * - count：当前已存字节数。
 *
 * 为什么需要 count：
 *   head == tail 在“空”和“满”两种情况下都会出现（写指针绕一圈追上读指针），
 *   仅凭两个指针无法区分，所以额外维护 count：
 *       count == 0      -> 空
 *       count == length -> 满（此时拒绝写入，绝不覆盖未读数据）
 *
 * 指针回绕：
 *   head/tail 增加到 length 时回到 0。因此一段连续数据/空间可能在数组末尾
 *   被“截断”成两段，Put_Buffer()/Get_Buffer() 内部会自动分两次 memcpy。
 *
 * 类型选择：
 *   length/head/tail/count 都是 uint16_t，容量上限 65535 字节。
 *   内部计算“指针 + 偏移”时会临时提升到 uint32_t 再回绕，
 *   避免 uint16_t 先溢出造成指针错位。
 */
typedef struct {
    uint8_t  *address;  /* 底层数组首地址（由调用者提供） */
    uint16_t length;    /* 缓冲区总容量，最大 65535 */
    uint16_t head;      /* 读指针（出队位置） */
    uint16_t tail;      /* 写指针（入队位置） */
    uint16_t count;     /* 当前已用字节数 */
} circle_buffer_t;

/* 初始化：把外部数组与结构体绑定，head/tail/count 清零（不复制数组内容） */
circle_buffer_t Circle_Buffer_Init(uint8_t *array, uint16_t length);

/* 单字节读写：Put 在满时返回 false；Get 在空时返回调用者给的默认值 */
bool Circle_Buffer_Put(circle_buffer_t *buffer, uint8_t x);
uint8_t Circle_Buffer_Get(circle_buffer_t *buffer, uint8_t default_val);

/*
 * 整块读写：返回实际处理的字节数（uint16_t，最大 65535）。
 * 空间/数据不足时只处理能容纳的部分（不会越界，也不需要调用者处理回绕）。
 */
uint16_t Circle_Buffer_Put_Buffer(circle_buffer_t *buffer, const uint8_t *data, uint16_t length);
uint16_t Circle_Buffer_Get_Buffer(circle_buffer_t *buffer, uint8_t *data, uint16_t length);

/*
 * 只读查看距队首 offset 处的字节，不出队、不修改任何指针。
 * 用于扫描包头/包尾：先“看”清楚帧在哪里，确认后再决定是否 Get/Drop。
 */
bool Circle_Buffer_Peek(const circle_buffer_t *buffer, uint16_t offset, uint8_t *data);

/*
 * 直接丢弃队首 length 字节。
 * 典型用途：一帧已通过 DMA 完整发出后，才释放它在发送缓冲区里占用的空间，
 * 保证 DMA 读取期间数据不会被新数据覆盖。
 */
void Circle_Buffer_Drop(circle_buffer_t *buffer, uint16_t length);

/* 清空：只把 count/head/tail 归零，不擦除数组内容 */
void Circle_Buffer_Clear(circle_buffer_t *buffer);

/* 状态查询：Used 为已用字节数，Free 为还能写入的字节数（最大 65535） */
uint16_t Circle_Buffer_Used(const circle_buffer_t *buffer);
uint16_t Circle_Buffer_Free(const circle_buffer_t *buffer);
bool Circle_Buffer_IsEmpty(const circle_buffer_t *buffer);
bool Circle_Buffer_IsFull(const circle_buffer_t *buffer);

/* 兼容旧接口：返回已用字节数（等价于 Circle_Buffer_Used） */
uint16_t Circle_Buffer_Length(circle_buffer_t *buffer);

#endif /* __CIRCLE_BUFFER_H__ */
