#include "circle_buffer.h"
#include <string.h>

/* ==================== 内部私有函数 ==================== */

/* 读指针前进 (出队) */
static bool Circle_Buffer_HeadMove(circle_buffer_t *buffer)
{
    if (buffer == NULL || buffer->length == 0U || buffer->count == 0U) return false;

    buffer->head++;
    if (buffer->head >= buffer->length) buffer->head = 0U;
    buffer->count--;
    return true;
}

/* 写指针前进 (入队) */
static bool Circle_Buffer_TailMove(circle_buffer_t *buffer)
{
    if (buffer == NULL || buffer->length == 0U || buffer->count >= buffer->length) return false;

    buffer->tail++;
    if (buffer->tail >= buffer->length) buffer->tail = 0U;
    buffer->count++;
    return true;
}

/* ==================== 对外公开的 API ==================== */

/**
 * @brief 初始化环形缓冲区
 * @param array  外部传入的实体数组
 * @param length 数组长度（uint16_t，最大 65535）
 * @return 初始化好的缓冲区结构体
 */
circle_buffer_t Circle_Buffer_Init(uint8_t *array, uint16_t length)
{
    circle_buffer_t buffer;

    buffer.address = array;
    buffer.length  = (array == NULL) ? 0U : length;
    buffer.head    = 0U;
    buffer.tail    = 0U;
    buffer.count   = 0U;
    return buffer;
}

/**
 * @brief 向缓冲区写入一个字节
 * @return true 写入成功；false 缓冲区已满或参数非法
 */
bool Circle_Buffer_Put(circle_buffer_t *buffer, uint8_t x)
{
    if (buffer == NULL || buffer->length == 0U || buffer->count >= buffer->length) {
        return false;   /* 满了，丢弃新数据 */
    }

    buffer->address[buffer->tail] = x;
    return Circle_Buffer_TailMove(buffer);
}

/**
 * @brief 从缓冲区读取一个字节
 * @param default_val 缓冲区为空时返回的默认值
 */
uint8_t Circle_Buffer_Get(circle_buffer_t *buffer, uint8_t default_val)
{
    if (buffer == NULL || buffer->length == 0U || buffer->count == 0U) {
        return default_val;
    }

    uint8_t data = buffer->address[buffer->head];
    (void)Circle_Buffer_HeadMove(buffer);
    return data;
}

/**
 * @brief 整块写入：空间不足时只写入能容纳的部分
 * @return 实际写入的字节数
 */
uint16_t Circle_Buffer_Put_Buffer(circle_buffer_t *buffer, const uint8_t *data, uint16_t length)
{
    if (buffer == NULL || data == NULL || buffer->length == 0U) return 0U;

    uint16_t written = 0U;
    while (written < length) {
        uint16_t free_space = (uint16_t)(buffer->length - buffer->count);
        if (free_space == 0U) break;

        uint16_t to_end = (uint16_t)(buffer->length - buffer->tail);  /* tail 到数组末尾的连续空间 */
        uint16_t chunk  = (uint16_t)(length - written);
        if (chunk > free_space) chunk = free_space;
        if (chunk > to_end)     chunk = to_end;

        (void)memcpy(&buffer->address[buffer->tail], &data[written], chunk);
        uint32_t next_tail = (uint32_t)buffer->tail + chunk;
        if (next_tail >= buffer->length) next_tail -= buffer->length;
        buffer->tail = (uint16_t)next_tail;
        buffer->count = (uint16_t)(buffer->count + chunk);
        written = (uint16_t)(written + chunk);
    }
    return written;
}

/**
 * @brief 整块读出：数据不足时只读出已有的部分
 * @return 实际读出的字节数
 */
uint16_t Circle_Buffer_Get_Buffer(circle_buffer_t *buffer, uint8_t *data, uint16_t length)
{
    if (buffer == NULL || data == NULL || buffer->length == 0U) return 0U;

    uint16_t read_len = 0U;
    while (read_len < length) {
        if (buffer->count == 0U) break;

        uint16_t to_end = (uint16_t)(buffer->length - buffer->head); /* head 到数组末尾的连续数据 */
        uint16_t chunk  = (uint16_t)(length - read_len);
        if (chunk > buffer->count) chunk = buffer->count;
        if (chunk > to_end)        chunk = to_end;

        (void)memcpy(&data[read_len], &buffer->address[buffer->head], chunk);
        uint32_t next_head = (uint32_t)buffer->head + chunk;
        if (next_head >= buffer->length) next_head -= buffer->length;
        buffer->head = (uint16_t)next_head;
        buffer->count = (uint16_t)(buffer->count - chunk);
        read_len = (uint16_t)(read_len + chunk);
    }
    return read_len;
}

/**
 * @brief 只读查看距队首 offset 处的字节，不移动读指针
 */
bool Circle_Buffer_Peek(const circle_buffer_t *buffer, uint16_t offset, uint8_t *data)
{
    if (buffer == NULL || data == NULL || buffer->length == 0U || offset >= buffer->count) return false;

    uint32_t index = (uint32_t)buffer->head + offset;
    if (index >= buffer->length) index -= buffer->length;
    *data = buffer->address[index];
    return true;
}

/**
 * @brief 丢弃队首 length 字节
 */
void Circle_Buffer_Drop(circle_buffer_t *buffer, uint16_t length)
{
    if (buffer == NULL || buffer->length == 0U) return;

    if (length > buffer->count) length = buffer->count;
    uint32_t next_head = (uint32_t)buffer->head + length;
    if (next_head >= buffer->length) next_head -= buffer->length;
    buffer->head = (uint16_t)next_head;
    buffer->count = (uint16_t)(buffer->count - length);
}

/**
 * @brief 清空缓冲区（不清数组内容，count 为 0 即可）
 */
void Circle_Buffer_Clear(circle_buffer_t *buffer)
{
    if (buffer == NULL) return;

    buffer->head  = 0U;
    buffer->tail  = 0U;
    buffer->count = 0U;
}

uint16_t Circle_Buffer_Used(const circle_buffer_t *buffer)
{
    if (buffer == NULL || buffer->length == 0U) return 0U;
    return buffer->count;
}

uint16_t Circle_Buffer_Free(const circle_buffer_t *buffer)
{
    if (buffer == NULL || buffer->length == 0U) return 0U;
    return (uint16_t)(buffer->length - buffer->count);
}

bool Circle_Buffer_IsEmpty(const circle_buffer_t *buffer)
{
    return (buffer == NULL) || (buffer->count == 0U);
}

bool Circle_Buffer_IsFull(const circle_buffer_t *buffer)
{
    return (buffer != NULL) && (buffer->length != 0U) && (buffer->count >= buffer->length);
}

/* 兼容旧接口：返回已用长度 */
uint16_t Circle_Buffer_Length(circle_buffer_t *buffer)
{
    return Circle_Buffer_Used(buffer);
}

