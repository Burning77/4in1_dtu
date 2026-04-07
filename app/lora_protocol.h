#ifndef __LORA_PROTOCOL_H__
#define __LORA_PROTOCOL_H__

#include <stdint.h>
#include <stddef.h>

// 前向声明
typedef struct fifo_message_t fifo_message_t;

/**
 * @brief LoRa协议处理器接口
 * 
 * 任何LoRa组网协议都应实现此接口。
 * 系统将调用这些方法对原始数据进行封装/解析。
 */
typedef struct {
    /**
     * @brief 获取协议的唯一标识符
     * @return 协议ID（例如：0x01表示自定义协议1）
     */
    uint8_t (*get_protocol_id)(void);
    
    /**
     * @brief 获取协议名称（用于日志）
     */
    const char* (*get_protocol_name)(void);
    
    /**
     * @brief 封装应用数据为LoRa帧
     * 
     * @param app_data 应用层原始数据
     * @param app_len  应用数据长度
     * @param lora_buf 输出缓冲区（存放封装后的LoRa帧）
     * @param buf_size 输出缓冲区大小
     * @return 封装后的数据长度（≤0表示失败）
     */
    int (*packet_encoder)(const uint8_t* app_data, int app_len, 
                          uint8_t* lora_buf, int buf_size);
    
    /**
     * @brief 解析LoRa帧为应用数据
     * 
     * @param lora_data 接收到的LoRa帧
     * @param lora_len  LoRa帧长度
     * @param app_buf   输出缓冲区（存放解析后的应用数据）
     * @param buf_size  输出缓冲区大小
     * @return 解析出的应用数据长度（≤0表示解析失败）
     */
    int (*packet_decoder)(const uint8_t* lora_data, int lora_len,
                          uint8_t* app_buf, int buf_size);
    
    /**
     * @brief 判断该协议是否需要确认（ACK）
     * 
     * @param lora_data LoRa帧数据
     * @param lora_len  数据长度
     * @return 1需要ACK，0不需要
     */
    int (*need_ack)(const uint8_t* lora_data, int lora_len);
    
    /**
     * @brief 生成ACK响应帧
     * 
     * @param original_frame 原始请求帧
     * @param frame_len      原始帧长度
     * @param ack_buf        ACK帧缓冲区
     * @param buf_size       缓冲区大小
     * @return ACK帧长度
     */
    int (*generate_ack)(const uint8_t* original_frame, int frame_len,
                        uint8_t* ack_buf, int buf_size);
} lora_protocol_t;

/**
 * @brief 全局协议处理器指针
 * 
 * 在main函数中初始化为具体的协议实现。
 * 如果为NULL，则使用原始数据发送（无协议封装）。
 */
extern lora_protocol_t* active_lora_protocol;

/**
 * @brief 注册协议处理器
 * 
 * @param protocol 协议处理器实例
 * @return 0成功，-1失败
 */
int register_lora_protocol(lora_protocol_t* protocol);

/**
 * @brief 取消注册协议处理器
 */
void unregister_lora_protocol(void);

/**
 * @brief 通过协议发送数据（应用层接口）
 * 
 * 此函数将自动调用当前注册的协议处理器进行封装，
 * 然后调用底层Lora_send发送。
 * 
 * @param app_data 应用层数据
 * @param app_len  数据长度
 * @param msg_type 消息类型（用于日志/统计）
 * @return 0成功，<0失败
 */
int lora_send_with_protocol(const uint8_t* app_data, int app_len, 
                            const char* msg_type);

/**
 * @brief 处理接收到的LoRa数据
 * 
 * 从底层驱动接收原始LoRa数据，通过协议解析后
 * 放入数据队列供其他线程处理。
 * 
 * @param lora_data 原始LoRa数据
 * @param lora_len  数据长度
 */
void lora_receive_handler(uint8_t* lora_data, int lora_len);

#endif // __LORA_PROTOCOL_H__