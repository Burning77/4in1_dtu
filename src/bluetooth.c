/**
 * @file bluetooth.c
 * @brief 蓝牙模块实现
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <gpiod.h>
#include <stdatomic.h>
#include "../inc/bluetooth.h"
#include "../inc/usart.h"

// ============== 外部变量 ==============
extern int bt_fd;
extern atomic_int stop_flag;
extern struct gpiod_line *line_bt_status;

// ============== 全局变量 ==============
static send_path_t g_send_path = SEND_PATH_AUTO;  // 默认自动选择
static pthread_mutex_t g_path_mutex = PTHREAD_MUTEX_INITIALIZER;

// DTU状态（由其他模块更新）
static dtu_status_t g_dtu_status = {0};
static pthread_mutex_t g_status_mutex = PTHREAD_MUTEX_INITIALIZER;

// 启动时间
static time_t g_start_time = 0;

// ============== 内部函数 ==============

/**
 * @brief 计算校验和（XOR）
 */
static unsigned char calc_checksum(const char *s)
{
    unsigned char checksum = 0;
    const char *p = strchr(s, '$');
    if (p) p++;
    while (p && *p && *p != '*') {
        checksum ^= *p;
        p++;
    }
    return checksum;
}

/**
 * @brief 验证校验和
 */
static int verify_checksum(const char *buf)
{
    const char *star = strchr(buf, '*');
    if (!star) return 0;
    
    unsigned char expected = (unsigned char)strtol(star + 1, NULL, 16);
    unsigned char calculated = calc_checksum(buf);
    
    return (expected == calculated);
}

// ============== 公共函数实现 ==============

int bt_init(void)
{
    g_start_time = time(NULL);
    g_send_path = SEND_PATH_AUTO;
    
    printf("[BT] Bluetooth module initialized\n");
    printf("[BT] Default send path: AUTO\n");
    
    return 0;
}

int bt_is_connected(void)
{
    if (line_bt_status == NULL) {
        return 0;
    }
    // BT_STATUS GPIO: 高电平表示已连接
    int value = gpiod_line_get_value(line_bt_status);
    return (value == 1) ? 1 : 0;
}

send_path_t bt_get_send_path(void)
{
    send_path_t path;
    pthread_mutex_lock(&g_path_mutex);
    path = g_send_path;
    pthread_mutex_unlock(&g_path_mutex);
    return path;
}

void bt_set_send_path(send_path_t path)
{
    pthread_mutex_lock(&g_path_mutex);
    g_send_path = path;
    pthread_mutex_unlock(&g_path_mutex);
    
    const char *path_names[] = {"AUTO", "4G_ONLY", "BD_ONLY", "4G_FIRST", "BD_FIRST"};
    printf("[BT] Send path changed to: %s\n", path_names[path]);
}

int bt_send_heartbeat(const dtu_status_t *status)
{
    char msg[128];
    uint32_t uptime = (uint32_t)(time(NULL) - g_start_time);
    
    // 格式: $HB,<4g_status>,<4g_signal>,<bt_status>,<send_path>,<uptime>*<checksum>\r\n
    int len = snprintf(msg, sizeof(msg) - 5, 
                       "$HB,%d,%d,%d,%d,%u*",
                       status->eg_connected,
                       status->eg_signal,
                       status->bt_connected,
                       status->send_path,
                       uptime);
    
    unsigned char cs = calc_checksum(msg);
    snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);
    
    int ret = data_send((unsigned char *)msg, strlen(msg), BT_DEV);
    if (ret > 0) {
        // printf("[BT] Heartbeat sent: %s", msg);
        return 0;
    }
    return -1;
}

int bt_send_status(const dtu_status_t *status)
{
    char msg[256];
    uint32_t uptime = (uint32_t)(time(NULL) - g_start_time);
    
    // 格式: $STATUS,<4g>,<4g_sig>,<bt>,<path>,<uptime>,<cnt_4g>,<cnt_bd>,<fail>*<cs>\r\n
    int len = snprintf(msg, sizeof(msg) - 5,
                       "$STATUS,%d,%d,%d,%d,%u,%u,%u,%u*",
                       status->eg_connected,
                       status->eg_signal,
                       status->bt_connected,
                       status->send_path,
                       uptime,
                       status->send_count_4g,
                       status->send_count_bd,
                       status->send_fail_count);
    
    unsigned char cs = calc_checksum(msg);
    snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);
    
    int ret = data_send((unsigned char *)msg, strlen(msg), BT_DEV);
    if (ret > 0) {
        printf("[BT] Status sent: %s", msg);
        return 0;
    }
    return -1;
}

int bt_send_response(uint8_t cmd_id, uint8_t result, const char *data)
{
    char msg[128];
    int len;
    
    if (data && strlen(data) > 0) {
        len = snprintf(msg, sizeof(msg) - 5, "$RSP,%02X,%02X,%s*", cmd_id, result, data);
    } else {
        len = snprintf(msg, sizeof(msg) - 5, "$RSP,%02X,%02X*", cmd_id, result);
    }
    
    unsigned char cs = calc_checksum(msg);
    snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);
    
    int ret = data_send((unsigned char *)msg, strlen(msg), BT_DEV);
    if (ret > 0) {
        printf("[BT] Response sent: %s", msg);
        return 0;
    }
    return -1;
}

int bt_parse_command(const char *buf, int len, uint8_t *cmd_id, uint8_t *param)
{
    // 命令格式: $CMD,<cmd_id>,<param>*<checksum>\r\n
    if (len < 10 || buf[0] != '$') {
        return -1;
    }
    
    // 验证校验和
    if (!verify_checksum(buf)) {
        printf("[BT] Checksum verification failed\n");
        return -1;
    }
    
    // 解析命令
    unsigned int id, p;
    if (sscanf(buf, "$CMD,%x,%x*", &id, &p) == 2) {
        *cmd_id = (uint8_t)id;
        *param = (uint8_t)p;
        return 0;
    }
    
    // 尝试无参数格式
    if (sscanf(buf, "$CMD,%x*", &id) == 1) {
        *cmd_id = (uint8_t)id;
        *param = 0;
        return 0;
    }
    
    return -1;
}

/**
 * @brief 处理蓝牙命令
 */
static void bt_handle_command(uint8_t cmd_id, uint8_t param)
{
    printf("[BT] Received command: 0x%02X, param: 0x%02X\n", cmd_id, param);
    
    switch (cmd_id) {
        case BT_CMD_SET_PATH:
            if (param <= SEND_PATH_BD_FIRST) {
                bt_set_send_path((send_path_t)param);
                bt_send_response(cmd_id, BT_RSP_OK, NULL);
            } else {
                bt_send_response(cmd_id, BT_RSP_ERR, "Invalid path");
            }
            break;
            
        case BT_CMD_GET_STATUS:
            {
                dtu_status_t status;
                pthread_mutex_lock(&g_status_mutex);
                memcpy(&status, &g_dtu_status, sizeof(status));
                pthread_mutex_unlock(&g_status_mutex);
                bt_send_status(&status);
            }
            break;
            
        case BT_CMD_GET_CONFIG:
            {
                char config[64];
                snprintf(config, sizeof(config), "PATH=%d", bt_get_send_path());
                bt_send_response(cmd_id, BT_RSP_OK, config);
            }
            break;
            
        case BT_CMD_REBOOT:
            bt_send_response(cmd_id, BT_RSP_OK, "Rebooting...");
            sleep(1);
            // system("reboot");  // 取消注释以启用重启功能
            printf("[BT] Reboot command received (disabled)\n");
            break;
            
        default:
            bt_send_response(cmd_id, BT_RSP_INVALID, NULL);
            break;
    }
}

/**
 * @brief 更新DTU状态（供其他模块调用）
 */
void bt_update_status(uint8_t eg_connected, uint8_t eg_signal,
                      uint32_t send_count_4g, uint32_t send_count_bd,
                      uint32_t send_fail_count)
{
    pthread_mutex_lock(&g_status_mutex);
    g_dtu_status.eg_connected = eg_connected;
    g_dtu_status.eg_signal = eg_signal;
    g_dtu_status.bt_connected = bt_is_connected();
    g_dtu_status.send_path = bt_get_send_path();
    g_dtu_status.uptime = (uint32_t)(time(NULL) - g_start_time);
    g_dtu_status.send_count_4g = send_count_4g;
    g_dtu_status.send_count_bd = send_count_bd;
    g_dtu_status.send_fail_count = send_fail_count;
    pthread_mutex_unlock(&g_status_mutex);
}

/**
 * @brief 蓝牙通信线程
 */
void *bt_comm_thread(void *arg)
{
    (void)arg;
    
    char recv_buf[256] = {0};
    int recv_len = 0;
    time_t last_heartbeat = 0;
    fd_set fds;
    struct timeval tv;
    
    printf("[BT] Communication thread started\n");
    
    // 初始化蓝牙模块
    bt_init();
    
    while (!stop_flag) {
        // 检查是否需要发送心跳
        time_t now = time(NULL);
        if (now - last_heartbeat >= BT_HEARTBEAT_INTERVAL) {
            dtu_status_t status;
            pthread_mutex_lock(&g_status_mutex);
            memcpy(&status, &g_dtu_status, sizeof(status));
            status.bt_connected = bt_is_connected();
            status.send_path = bt_get_send_path();
            pthread_mutex_unlock(&g_status_mutex);
            
            bt_send_heartbeat(&status);
            last_heartbeat = now;
        }
        
        // 检查蓝牙数据
        FD_ZERO(&fds);
        FD_SET(bt_fd, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        
        int ret = select(bt_fd + 1, &fds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(bt_fd, &fds)) {
            int n = data_recv(recv_buf + recv_len, sizeof(recv_buf) - 1 - recv_len, BT_DEV);
            if (n > 0) {
                recv_len += n;
                recv_buf[recv_len] = '\0';
                
                // 查找完整命令（以\r\n结尾）
                char *end = strstr(recv_buf, "\r\n");
                while (end != NULL) {
                    *end = '\0';
                    
                    // 解析并处理命令
                    uint8_t cmd_id, param;
                    if (bt_parse_command(recv_buf, end - recv_buf, &cmd_id, &param) == 0) {
                        bt_handle_command(cmd_id, param);
                    } else {
                        printf("[BT] Invalid command: %s\n", recv_buf);
                    }
                    
                    // 移动剩余数据
                    int remaining = recv_len - (end - recv_buf + 2);
                    if (remaining > 0) {
                        memmove(recv_buf, end + 2, remaining);
                        recv_len = remaining;
                    } else {
                        recv_len = 0;
                    }
                    recv_buf[recv_len] = '\0';
                    
                    end = strstr(recv_buf, "\r\n");
                }
                
                // 防止缓冲区溢出
                if (recv_len > 200) {
                    recv_len = 0;
                    recv_buf[0] = '\0';
                }
            }
        }
    }
    
    printf("[BT] Communication thread stopped\n");
    return NULL;
}
