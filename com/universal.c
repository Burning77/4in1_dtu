#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "../inc/universal.h"
#include "string.h"
#include "../inc/usart.h"
#include "../inc/kfifo.h"

#define FRAME_TIMEOUT_MS 10
#define BUFFER_SIZE 1024

extern struct gpiod_line *line_bd_en;
extern struct gpiod_line *line_bd_pow;
extern struct gpiod_line *line_bt_pow;
extern struct gpiod_line *line_4g_pow;


unsigned char calc_checksum(const char *s)
{
    unsigned char checksum = 0;
    // ��$��ĵ�һ���ַ����?
    const char *p = strchr(s, '$');
    if (p)
        p++; // ����$
    while (p && *p && *p != '*')
    {
        checksum ^= *p;
        p++;
    }
    return checksum;
}

int parse_log_line(const char *line, unsigned char *out_data, int max_len)
{
    // 跳过时间戳部分：找到 ']' 后的空格
    char *data_start = strchr(line, ']');
    if (!data_start)
        return 0;
    data_start++; // 跳过 ']'
    while (*data_start == ' ')
        data_start++;

    // 解析十六进制字节
    int count = 0;
    char *p = data_start;
    while (*p && count < max_len)
    {
        // 跳过空格
        while (*p == ' ')
            p++;
        if (!*p)
            break;
        // 提取两个十六进制字符
        unsigned int byte;
        if (sscanf(p, "%2x", &byte) != 1)
            break;
        out_data[count++] = (unsigned char)byte;
        p += 2;
    }
    return count;
}

void rf_power_on(void)
{
    gpio_set_value(1, line_bd_en);
    gpio_set_value(1, line_4g_pow);
    gpio_set_value(0, line_bt_pow);
}

// 静态函数：提交一帧数据
static void submit_frame(serial_state_t* state, frame_processor_ctx_t* ctx, const char* reason) {
    if (state->frame_len <= 0) {
        return;
    }

    // 构造消息
    fifo_message_t msg;
    msg.type = state->data_type;
    msg.len = state->frame_len;
    
    // 安全检查：确保不会溢出
    int copy_len = state->frame_len;
    if (copy_len > BUFFER_SIZE) {
        copy_len = BUFFER_SIZE;
    }
    
    memcpy(msg.data, state->frame_buf, copy_len);
    
    // 加锁推入 kfifo
    pthread_mutex_lock(ctx->fifo_lock);
    kfifo_put(ctx->fifo, (unsigned char *)&msg, sizeof(fifo_message_t));
    pthread_cond_signal(ctx->fifo_not_empty);
    pthread_mutex_unlock(ctx->fifo_lock);
    
    // 打印调试信息
    printf("[%s COMPLETE %d bytes (%s)]: ", state->tag, state->frame_len, reason);
    for (int i = 0; i < state->frame_len && i < 32; i++) { // 限制打印长度
        printf("%02X ", state->frame_buf[i]);
    }
    if (state->frame_len > 32) {
        printf("... (+%d more bytes)", state->frame_len - 32);
    }
    printf("\n");
}

// 初始化串口状态
void serial_state_init(serial_state_t* state, int data_type, const char* tag) {
    if (!state) return;
    
    memset(state->frame_buf, 0, sizeof(state->frame_buf));
    state->frame_len = 0;
    state->last_recv.tv_sec = 0;
    state->last_recv.tv_usec = 0;
    state->data_type = data_type;
    state->tag = tag;
}

// 处理串口数据
void process_serial_data(serial_state_t* state,
                         unsigned char* read_buf, int read_len,
                         int is_timeout_triggered,
                         frame_processor_ctx_t* ctx) {
    if (!state || !ctx) {
        return;
    }
    
    struct timeval now;
    gettimeofday(&now, NULL);
    
    // 计算距离上次接收的时间差（毫秒）
    long diff_ms = 0;
    if (state->frame_len > 0) {
        diff_ms = (now.tv_sec - state->last_recv.tv_sec) * 1000 +
                  (now.tv_usec - state->last_recv.tv_usec) / 1000;
    }
    if (state->frame_len > 0 && diff_ms > FRAME_TIMEOUT_MS) {
        submit_frame(state, ctx, "timeout");
        state->frame_len = 0;
        // 提交后，如果本次有 read_len > 0，下面会处理新数据
    }
    if (state->frame_len >= MAX_LOG_LEN) {
        submit_frame(state, ctx, "max_len");
        state->frame_len = 0;
    }
    
    // 情况A: 有新数据到达
    if (read_len > 0) {
        // 检查缓冲区是否有足够空间
        if (state->frame_len + read_len < BUFFER_SIZE) {
            memcpy(state->frame_buf + state->frame_len, read_buf, read_len);
            state->frame_len += read_len;
            state->last_recv = now; // 更新最后接收时间
            if (state->frame_len >= MAX_LOG_LEN) {
                submit_frame(state, ctx, "max_len");
                state->frame_len = 0;
            }
        } else {
            // 缓冲区溢出，提交当前帧
            submit_frame(state, ctx, "overflow");
            state->frame_len = 0;
            // 尝试存储新数据
            if (read_len < BUFFER_SIZE) {
                memcpy(state->frame_buf, read_buf, read_len);
                state->frame_len = read_len;
                state->last_recv = now;
            }
        }
        return;
    }
    return;
}

// 强制提交缓冲区中的残留帧
void flush_serial_state(serial_state_t* state, frame_processor_ctx_t* ctx) {
    if (!state || !ctx) {
        return;
    }
    
    if (state->frame_len > 0) {
        submit_frame(state, ctx, "flush");
        state->frame_len = 0;
    }
}
