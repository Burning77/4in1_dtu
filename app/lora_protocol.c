#include "lora_protocol.h"
#include "../inc/llcc68.h"
#include "../inc/kfifo.h"
#include <pthread.h>
#include <string.h>
#include <stdio.h>

// 全局协议处理器（默认为原始数据模式）
lora_protocol_t *active_lora_protocol = NULL;
static pthread_mutex_t protocol_mutex = PTHREAD_MUTEX_INITIALIZER;

// 统计信息
typedef struct
{
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t protocol_errors;
} lora_stats_t;
static lora_stats_t lora_stats = {0};
static pthread_mutex_t stats_mutex = PTHREAD_MUTEX_INITIALIZER;

int register_lora_protocol(lora_protocol_t *protocol)
{
    if (!protocol || !protocol->packet_encoder || !protocol->packet_decoder)
    {
        fprintf(stderr, "[LORA_PROTOCOL] Invalid protocol handler\n");
        return -1;
    }

    pthread_mutex_lock(&protocol_mutex);
    if (active_lora_protocol)
    {
        fprintf(stderr, "[LORA_PROTOCOL] Protocol already registered\n");
        pthread_mutex_unlock(&protocol_mutex);
        return -1;
    }

    active_lora_protocol = protocol;
    printf("[LORA_PROTOCOL] Registered protocol: %s (ID: 0x%02X)\n",
           protocol->get_protocol_name ? protocol->get_protocol_name() : "Unknown",
           protocol->get_protocol_id ? protocol->get_protocol_id() : 0xFF);
    pthread_mutex_unlock(&protocol_mutex);
    return 0;
}

void unregister_lora_protocol(void)
{
    pthread_mutex_lock(&protocol_mutex);
    if (active_lora_protocol)
    {
        printf("[LORA_PROTOCOL] Unregistered protocol: %s\n",
               active_lora_protocol->get_protocol_name ? active_lora_protocol->get_protocol_name() : "Unknown");
        active_lora_protocol = NULL;
    }
    pthread_mutex_unlock(&protocol_mutex);
}

int lora_send_with_protocol(const uint8_t *app_data, int app_len,
                            const char *msg_type)
{
    uint8_t lora_buffer[256]; // 根据协议最大帧长调整
    int frame_len;

    // 1. 协议封装
    pthread_mutex_lock(&protocol_mutex);
    if (active_lora_protocol && active_lora_protocol->packet_encoder)
    {
        frame_len = active_lora_protocol->packet_encoder(app_data, app_len,
                                                         lora_buffer, sizeof(lora_buffer));
        if (frame_len <= 0)
        {
            pthread_mutex_unlock(&protocol_mutex);
            fprintf(stderr, "[LORA_PROTOCOL] Packet encode failed for: %s\n",
                    msg_type ? msg_type : "Unknown");
            return -1;
        }
    }
    else
    {
        // 无协议封装，直接使用原始数据
        if (app_len > sizeof(lora_buffer))
        {
            pthread_mutex_unlock(&protocol_mutex);
            fprintf(stderr, "[LORA_PROTOCOL] Data too large: %d > %zu\n",
                    app_len, sizeof(lora_buffer));
            return -2;
        }
        memcpy(lora_buffer, app_data, app_len);
        frame_len = app_len;
    }
    pthread_mutex_unlock(&protocol_mutex);

    // 2. 调用底层驱动发送
    Lora_send(lora_buffer, frame_len);

    // 3. 更新统计
    pthread_mutex_lock(&stats_mutex);
    lora_stats.tx_packets++;
    lora_stats.tx_bytes += frame_len;
    pthread_mutex_unlock(&stats_mutex);

    printf("[LORA_TX] %s | Size: %d/%d bytes | Stats: TX=%u, RX=%u\n",
           msg_type ? msg_type : "Data", app_len, frame_len,
           lora_stats.tx_packets, lora_stats.rx_packets);
    return 0;
}

void lora_receive_handler(uint8_t *lora_data, int lora_len)
{
    uint8_t app_buffer[256];
    int app_len;

    if (lora_len <= 0)
        return;

    // 更新接收统计
    pthread_mutex_lock(&stats_mutex);
    lora_stats.rx_packets++;
    lora_stats.rx_bytes += lora_len;
    pthread_mutex_unlock(&stats_mutex);

    // 1. 协议解析
    pthread_mutex_lock(&protocol_mutex);
    if (active_lora_protocol && active_lora_protocol->packet_decoder)
    {
        app_len = active_lora_protocol->packet_decoder(lora_data, lora_len,
                                                       app_buffer, sizeof(app_buffer));
        if (app_len <= 0)
        {
            pthread_mutex_unlock(&protocol_mutex);

            pthread_mutex_lock(&stats_mutex);
            lora_stats.protocol_errors++;
            pthread_mutex_unlock(&stats_mutex);

            printf("[LORA_RX] Protocol decode failed | Raw: %d bytes\n", lora_len);
            return;
        }
    }
    else
    {
        // 无协议解析，直接使用原始数据
        app_len = (lora_len > sizeof(app_buffer)) ? sizeof(app_buffer) : lora_len;
        memcpy(app_buffer, lora_data, app_len);
    }
    pthread_mutex_unlock(&protocol_mutex);

    // 2. 检查是否需要ACK
    int need_ack = 0;
    uint8_t ack_buffer[64];
    int ack_len = 0;

    pthread_mutex_lock(&protocol_mutex);
    if (active_lora_protocol && active_lora_protocol->need_ack)
    {
        need_ack = active_lora_protocol->need_ack(lora_data, lora_len);
        if (need_ack && active_lora_protocol->generate_ack)
        {
            ack_len = active_lora_protocol->generate_ack(lora_data, lora_len,
                                                         ack_buffer, sizeof(ack_buffer));
        }
    }
    pthread_mutex_unlock(&protocol_mutex);

    // 3. 发送ACK（如果需要）
    if (need_ack && ack_len > 0)
    {
        Lora_send(ack_buffer, ack_len);
        printf("[LORA_ACK] Sent ACK, %d bytes\n", ack_len);
    }

    // 4. 将解析后的应用数据放入队列（供其他线程处理）
    // 这里可以根据实际需求将app_buffer放入另一个fifo
    printf("[LORA_RX] Decoded: %d bytes | Need ACK: %d\n", app_len, need_ack);

    // 示例：打印16进制
    printf("Hex: ");
    for (int i = 0; i < app_len; i++)
    {
        printf("%02X ", app_buffer[i]);
    }
    printf("\n");
}