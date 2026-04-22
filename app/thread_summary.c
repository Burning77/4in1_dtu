#include "../app/thread_summary.h"
#include "../inc/eg800k.h"
#include "../inc/watch_dog.h"
#include "../inc/bluetooth.h"
extern struct kfifo data_fifo;

static pthread_mutex_t fifo_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t fifo_not_empty = PTHREAD_COND_INITIALIZER;
extern volatile sig_atomic_t stop_flag;
extern int rtc_fd;
int data_num = 1;
static uint8_t rs485_frame_buf[1024];
static int rs485_frame_len = 0;
static struct timeval rs485_last_recv = {0};
static uint8_t rs232_frame_buf[1024];
static int rs232_frame_len = 0;
static struct timeval rs232_last_recv = {0};
static uint8_t bd_frame_buf[256];
static int bd_frame_len = 0;
static struct timeval bd_last_recv = {0};
static eg_state_t eg_state = EG_STATE_INIT;
// 4G 网络可用性标志（由监测线程更新）
static volatile int g_4g_available = 0; // 0=不可用, 1=可用
static pthread_mutex_t g_4g_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_pdp_deact = 0;
static pthread_mutex_t g_pdp_mutex = PTHREAD_MUTEX_INITIALIZER;
// 4G 初始化完成标志（防止 receive_thread 抢占 eg_fd 数据）
static volatile int g_eg_init_done = 0;
static pthread_mutex_t g_eg_init_mutex = PTHREAD_MUTEX_INITIALIZER;
// 发送统计
static uint32_t g_send_count_4g = 0;
static uint32_t g_send_count_bd = 0;
static uint32_t g_send_fail_count = 0;
extern int rs485_fd;
extern int rs232_fd;
extern int bd_fd;
extern int bt_fd;
extern int eg_fd;
extern int watchdog_fd;
void handle_signal(int sig)
{
    stop_flag = 1;
    pthread_cond_broadcast(&fifo_not_empty);
}

// 接收线程
void *receive_thread(void *arg)
{
    unsigned char buf[256];
    fd_set read_fds;
    struct timeval tv;
    int rs485_fd = get_fd(RS485_DEV);
    int rs232_fd = get_fd(RS232_DEV);
    int bd_fd = get_fd(BD_DEV);
    int max_fd;

    // 初始化串口状态
    serial_state_t rs485_state, rs232_state, bd_state, bt_state, eg_state;
    serial_state_init(&rs485_state, RS485_DATA, "RS485");
    serial_state_init(&rs232_state, RS232_DATA, "RS232");
    serial_state_init(&bd_state, BD_DATA, "BD");
    serial_state_init(&bt_state, BT_DATA, "BT");
    serial_state_init(&eg_state, EG_DATA, "EG");

    // 初始化帧处理器上下文
    frame_processor_ctx_t ctx = {
        .fifo = &data_fifo,
        .fifo_lock = &fifo_lock,
        .fifo_not_empty = &fifo_not_empty};

    while (!stop_flag)
    {
        FD_ZERO(&read_fds);
        FD_SET(rs485_fd, &read_fds);
        FD_SET(rs232_fd, &read_fds);
        FD_SET(bd_fd, &read_fds);
        FD_SET(bt_fd, &read_fds);
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms 超时
        max_fd = MAX(MAX(MAX(rs485_fd, rs232_fd), bd_fd), bt_fd);

        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);

        if (ret < 0)
        {
            if (errno == EINTR)
                continue;
            perror("select");
            break;
        }

        // 处理 RS485
        int rs485_read_len = 0;
        if (FD_ISSET(rs485_fd, &read_fds))
        {
            rs485_read_len = data_recv(buf, sizeof(buf) - 1, RS485_DEV);
        }
        process_serial_data(&rs485_state, buf, rs485_read_len, (ret == 0), &ctx);

        // 处理 RS232
        int rs232_read_len = 0;
        if (FD_ISSET(rs232_fd, &read_fds))
        {
            rs232_read_len = data_recv(buf, sizeof(buf) - 1, RS232_DEV);
        }
        process_serial_data(&rs232_state, buf, rs232_read_len, (ret == 0), &ctx);

        int bd_read_len = 0;
        if (FD_ISSET(bd_fd, &read_fds))
        {
            bd_read_len = data_recv(buf, sizeof(buf) - 1, BD_DEV);
        }
        process_serial_data(&bd_state, buf, bd_read_len, (ret == 0), &ctx);

        int bt_read_len = 0;
        if (FD_ISSET(bt_fd, &read_fds))
        {
            bt_read_len = data_recv(buf, sizeof(buf) - 1, BT_DEV);
        }
        process_serial_data(&bt_state, buf, bt_read_len, (ret == 0), &ctx);

        int eg_read_len = 0;
        if (FD_ISSET(eg_fd, &read_fds))
        {
            eg_read_len = data_recv(buf, sizeof(buf) - 1, EG_DEV);
            // 检查是否有 PDP 去激活 URC
            if (eg_read_len > 0)
            {
                buf[eg_read_len] = '\0';
                printf("[EG DEBUG] Received: [%s]\n", buf);
                char *urc = strstr((char *)buf, "+QIURC: \"pdpdeact\"");
                if (urc != NULL)
                {
                    pthread_mutex_lock(&g_pdp_mutex);
                    g_pdp_deact = 1;
                    pthread_mutex_unlock(&g_pdp_mutex);
                    printf("[EG] Received +QIURC: pdpdeact, will re-init PDP\n");
                }
            }
        }
        process_serial_data(&eg_state, buf, eg_read_len, (ret == 0), &ctx);
    }

    // 退出前处理残留帧
    flush_serial_state(&rs485_state, &ctx);
    flush_serial_state(&rs232_state, &ctx);
    flush_serial_state(&bd_state, &ctx);

    return NULL;
}

// 发送线程
void *serial_send_thread(void *arg)
{
    const char *test = "Hello from rs232!\r\n";
    const char *bt_test = "AT+VER=?\r\n";
    // 注意：不再从此线程发送 EG 命令，避免与 main_send_thread 的 eg_init() 冲突
    while (!stop_flag)
    {
        sleep(SEND_INTERVAL);
        if (stop_flag)
            break;

        printf("[SEND] Sending command...\n");
        int resualt = data_send(test, strlen(test), RS232_DEV);
        int ret = data_send(CMD_STRING, strlen(CMD_STRING), RS485_DEV);
        int res_bd = data_send(BD_CARD, strlen(BD_CARD), BD_DEV);
        int res_bt = data_send(bt_test, strlen(bt_test), BT_DEV);
        if (resualt < 0)
        {
            perror("rs232_send");
        }
        if (ret < 0)
        {
            perror("rs485_send");
        }
        if (res_bd < 0)
        {
            perror("bd_send");
        }
        if (res_bt < 0)
        {
            perror("bt_send");
        }
    }
    return NULL;
}
void *read_rtc_thread(void *arg)
{
    int year, month, day, hour, min, sec;
    struct rtc_time get_time;
    while (!stop_flag)
    {
        sleep(SEND_INTERVAL);
        if (stop_flag)
            break;
        if (rtc_get_time(rtc_fd, &get_time) == 0)
        {
            printf("Current RTC time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   get_time.tm_year + 1900, get_time.tm_mon + 1, get_time.tm_mday,
                   get_time.tm_hour, get_time.tm_min, get_time.tm_sec);
        }
        else
        {
            perror("rtc_get_time");
        }
    }
    return NULL;
}
void *write_file_thread(void *arg)
{
    fifo_message_t msg;
    FILE *fp_485 = NULL, *fp_232 = NULL, *fp_bd = NULL;
    char log_line[1024];

    while (!stop_flag)
    {
        pthread_mutex_lock(&fifo_lock);
        // 等待直到至少有一个完整消息可读
        while (kfifo_len(&data_fifo) < sizeof(fifo_message_t) && !stop_flag)
        {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 0; // 200ms 超时
            ts.tv_nsec += 200 * 1000000;
            if (ts.tv_nsec >= 1000000000)
            {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000;
            }
            pthread_cond_timedwait(&fifo_not_empty, &fifo_lock, &ts);
        }
        if (stop_flag)
        {
            pthread_mutex_unlock(&fifo_lock);
            break;
        }
        // 读取消息
        unsigned int read_len = kfifo_get(&data_fifo, (unsigned char *)&msg, sizeof(msg));
        pthread_mutex_unlock(&fifo_lock);

        if (read_len != sizeof(msg))
            continue; // 数据不完整，跳过

        // 选择文件 - 根据数据类型选择对应的日志文件
        FILE **fp = NULL;
        const char *path = NULL;
        switch (msg.type)
        {
        case RS485_DATA:
            fp = &fp_485;
            path = RS485_LOG_PATH;
            break;
        case RS232_DATA:
            fp = &fp_232;
            path = RS232_LOG_PATH;
            break;
        case BD_DATA:
            fp = &fp_bd;
            path = BD_LOG_PATH;
            break;
        default:
            printf("[WARN] Unknown data type: %d, skipping\n", msg.type);
            continue; // 跳过未知类型
        }

        if (*fp == NULL)
        {
            *fp = fopen(path, "a");
            if (!*fp)
            {
                perror("fopen");
                continue;
            }
        }

        // 构造时间戳行
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        int pos = snprintf(log_line, sizeof(log_line),
                           "[%04d-%02d-%02d %02d:%02d:%02d] ",
                           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
        for (int i = 0; i < msg.len && pos < (int)sizeof(log_line) - 5; i++)
        {
            pos += snprintf(log_line + pos, sizeof(log_line) - pos, "%02X ", msg.data[i]);
        }
        log_line[pos] = '\n';
        log_line[pos + 1] = '\0';

        fputs(log_line, *fp);
        fflush(*fp); // 可改为批量 flush
    }

    if (fp_485)
        fclose(fp_485);
    if (fp_232)
        fclose(fp_232);
    if (fp_bd)
        fclose(fp_bd);
    return NULL;
}
// void *bd_send_thread(void *arg)
// {
//     char hex_buf[BD_MSG_LEN];
//     char msg[512];
//     const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
//     off_t offsets[2] = {0, 0};
//     int hex_len, entry_count;
//     load_offsets(OFFSET_FILE_BD, offsets, 2);

//     // 等待日志文件被创建（最多等待30秒）
//     while (!stop_flag)
//     {
//         if (pack_data_from_files(paths, offsets, 2, BD_MSG_LEN,
//                                  hex_buf, &hex_len, &entry_count) == 0)
//         {

//             // 构造并发送报文
//             if (entry_count > 0)
//             {
//                 snprintf(msg, sizeof(msg), "$CCTCQ,4314513,2,1,2,,%s*", hex_buf);
//                 unsigned char cs = calc_checksum(msg);
//                 size_t len = strlen(msg);
//                 snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);
//                 data_send((unsigned char *)msg, strlen(msg), BD_DEV);
//                 printf("[SEND BDTX] %s", msg);
//                 save_offsets(OFFSET_FILE_BD, offsets, 2);
//             }
//             else
//             {
//                 // 文件存在但没有有效数据，不发送任何报文
//                 printf("[INFO] No valid data in log files, skipping BD transmission.\n");
//             }
//         }
//         else
//         {
//             printf("[ERROR] pack_data_from_files failed\n");
//         }
//         // 等待60秒
//         for (int i = 0; i < 60 && !stop_flag; i++)
//         {
//             sleep(1);
//         }
//     }
//     save_offsets(OFFSET_FILE_BD, offsets, 2);
//     return NULL;
// }
void *lora_transform_thread(void *arg)
{
    const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH}; // 从 RS485 和 RS232 日志发送
    off_t offsets[] = {0, 0};                               // 初始偏移量
    char hex_buf[LORA_MAX_HEX_LEN + 64];                    // 需定义 LORA_MAX_HEX_LEN
    int hex_len, entry_count;
    unsigned char buf[256];
    while (!stop_flag)
    {
        if (pack_data_from_files(paths, offsets, 2, LORA_MAX_HEX_LEN,
                                 hex_buf, &hex_len, &entry_count) == 0)
        {
            if (entry_count > 0)
            {
                // 构造 LoRa 负载（可根据实际协议调整）
                uint8_t payload[256];
                int payload_len = 0;
                for (int i = 0; i < hex_len && payload_len < sizeof(payload); i += 2)
                {
                    unsigned int byte;
                    sscanf(hex_buf + i, "%2x", &byte);
                    payload[payload_len++] = (uint8_t)byte;
                }
                Lora_send(payload, payload_len);
                printf("[SEND LORA] Sent %d bytes from RS485 log.\n", payload_len);
                save_offsets(OFFSET_FILE_LORA, offsets, 1);
            }
            else
            {
                printf("[INFO] No valid data in RS485 log for LoRa, skipping transmission.\n");
            }
        }
        else
        {
            printf("[ERROR] pack_data_from_files failed for LoRa\n");
        }
        // 等待30秒
        for (int i = 0; i < 30 && !stop_flag; i++)
        {
            sleep(1);
        }
    }
    save_offsets(OFFSET_FILE_LORA, offsets, 1);
    return NULL;
}
// 主发送线程（从文件读取数据并发送）
// void *eg_send_thread(void *arg)
// {
//     // 1. 初始化模块
//     if (eg_init() != 0)
//     {
//         printf("[EG] Initialization failed, thread exiting\n");
//         return NULL;
//     }

//     // 2. 建立 TCP 连接
//     if (eg_connect() != 0)
//     {
//         printf("[EG] TCP connect failed, thread exiting\n");
//         return NULL;
//     }
//     eg_state = EG_STATE_CONNECTED;

//     // 3. 准备文件读取和偏移量管理
//     const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
//     off_t offsets[2] = {0, 0}; // ✅ 真正的 off_t 数组
//     int hex_len, entry_count;
//     char hex_buf[EG_MSG_LEN + 64]; // ✅ 定义 EG_MAX_HEX_LEN（例如 512）

//     // 加载持久化偏移量
//     load_offsets(OFFSET_FILE_4G, offsets, 2);

//     while (!stop_flag)
//     {
//         // 打包数据
//         if (pack_data_from_files(paths, offsets, 2, EG_MSG_LEN,
//                                  hex_buf, &hex_len, &entry_count) == 0)
//         {
//             if (entry_count > 0)
//             {
//                 // ✅ 将十六进制字符串转换为原始二进制数据
//                 unsigned char raw_data[256];
//                 int raw_len = hex_to_bytes(hex_buf, raw_data, sizeof(raw_data));
//                 if (raw_len > 0)
//                 {
//                     if (eg_send_data(raw_data, raw_len) == 0)
//                     {
//                         save_offsets(OFFSET_FILE_4G, offsets, 2);
//                         printf("[EG SEND] Sent %d bytes from logs.\n", raw_len);
//                     }
//                     else
//                     {
//                         // 发送失败，尝试重连
//                         printf("[EG] Send failed, attempting reconnect...\n");
//                         eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
//                         if (eg_connect() == 0)
//                         {
//                             // 重连成功，重试发送当前数据
//                             if (eg_send_data(raw_data, raw_len) == 0)
//                             {
//                                 save_offsets(OFFSET_FILE_4G, offsets, 2);
//                                 printf("[EG SEND] Sent %d bytes after reconnect.\n", raw_len);
//                             }
//                             else
//                             {
//                                 printf("[EG] Retry send still failed, skip this data\n");
//                                 // 注意：这里不更新偏移量，下次会重试同一数据
//                             }
//                         }
//                         else
//                         {
//                             printf("[EG] Reconnect failed, will retry later\n");
//                             // 等待一段时间后继续循环
//                             sleep(30);
//                         }
//                     }
//                 }
//                 else
//                 {
//                     printf("[EG] Failed to convert hex string to bytes\n");
//                 }
//             }
//             else
//             {
//                 printf("[INFO] No valid data for EG, sleeping...\n");
//             }
//         }
//         else
//         {
//             printf("[ERROR] pack_data_from_files failed\n");
//         }

//         // 等待一段时间再检查新数据（避免空转）
//         for (int i = 0; i < 10 && !stop_flag; i++)
//         {
//             sleep(1);
//         }
//     }

//     // 清理
//     eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
//     eg_send_cmd("AT+QIDEACT=1\r\n", "OK", 40);
//     save_offsets(OFFSET_FILE_4G, offsets, 2);
//     return NULL;
// }

void *eg_monitor_thread(void *arg)
{
    // 等待 4G 初始化完成
    while (!stop_flag)
    {
        pthread_mutex_lock(&g_eg_init_mutex);
        int init_done = g_eg_init_done;
        pthread_mutex_unlock(&g_eg_init_mutex);
        if (init_done)
            break;
        sleep(1);
    }
    
    // TCP 连接成功后，不再主动检测网络状态
    // 因为 eg_is_network_available() 会发送 AT 命令，与数据发送产生冲突
    // 网络状态由 main_send_thread 根据发送结果自行判断
    while (!stop_flag)
    {
        // 仅监控 PDP 去激活事件（由 receive_thread 通过 URC 检测）
        // 不再主动发送 AT 命令检测网络
        sleep(30);
    }
    return NULL;
}
void *main_send_thread(void *arg)
{
    const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
    off_t offsets[2] = {0, 0};
    int hex_len, entry_count;
    char hex_buf[EG_MSG_LEN + 64];
    time_t last_bd_send_time = 0;
    const int BD_SEND_INTERVAL = 60;

    // 加载持久化偏移量
    load_offsets(OFFSET_FILE_MAIN, offsets, 2);

    // ========== 1. 4G 模块初始化与连接（带重试） ==========
    int eg_initialized = 0;
    int eg_connected = 0;
    while (!stop_flag && !eg_initialized)
    {
        printf("[MAIN] Initializing 4G module...\n");
        if (eg_init() == 0)
        {
            eg_initialized = 1;
            printf("[MAIN] 4G module initialized\n");
        }
        else
        {
            printf("[MAIN] 4G init failed, retry in 30 seconds...\n");
            for (int i = 0; i < 30 && !stop_flag; i++)
                sleep(1);
        }
    }
    while (!stop_flag && !eg_connected)
    {
        printf("[MAIN] Establishing TCP connection...\n");
        if (eg_connect() == 0)
        {
            eg_connected = 1;
            printf("[MAIN] TCP connected\n");
        }
        else
        {
            printf("[MAIN] TCP connect failed, retry in 30 seconds...\n");
            for (int i = 0; i < 30 && !stop_flag; i++)
                sleep(1);
        }
    }

    // 标记 4G 初始化完成，允许 eg_monitor_thread 开始工作
    pthread_mutex_lock(&g_eg_init_mutex);
    g_eg_init_done = 1;
    pthread_mutex_unlock(&g_eg_init_mutex);

    // TCP 连接成功后，立即标记 4G 可用
    pthread_mutex_lock(&g_4g_mutex);
    g_4g_available = 1;
    pthread_mutex_unlock(&g_4g_mutex);

    // ========== 2. 主循环：发送数据 ==========
    while (!stop_flag)
    {
        // 检查 PDP 去激活标志（由接收线程设置）
        pthread_mutex_lock(&g_pdp_mutex);
        int need_reinit_pdp = g_pdp_deact;
        pthread_mutex_unlock(&g_pdp_mutex);
        if (need_reinit_pdp)
        {
            printf("[MAIN] PDP deactivated by network, re-initializing...\n");
            eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
            if (eg_reinit_pdp() == 0)
            {
                pthread_mutex_lock(&g_pdp_mutex);
                g_pdp_deact = 0;
                pthread_mutex_unlock(&g_pdp_mutex);
                eg_connected = 1;
                pthread_mutex_lock(&g_4g_mutex);
                g_4g_available = 1;
                pthread_mutex_unlock(&g_4g_mutex);
            }
            else
            {
                eg_connected = 0;
                pthread_mutex_lock(&g_4g_mutex);
                g_4g_available = 0;
                pthread_mutex_unlock(&g_4g_mutex);
                sleep(30);
                continue;
            }
        }

        // 获取 4G 可用性
        pthread_mutex_lock(&g_4g_mutex);
        int current_4g_avail = g_4g_available;
        pthread_mutex_unlock(&g_4g_mutex);

        // 如果 4G 可用但未连接，尝试重新连接
        if (current_4g_avail && !eg_connected)
        {
            printf("[MAIN] 4G available but not connected, trying to reconnect...\n");
            if (eg_connect() == 0)
            {
                eg_connected = 1;
                printf("[MAIN] 4G reconnected successfully\n");
            }
            else
            {
                printf("[MAIN] 4G reconnect failed, will retry later\n");
            }
        }

        // 获取蓝牙设置的发送路径
        send_path_t send_path = bt_get_send_path();
        int can_use_4g = (current_4g_avail && eg_connected);
        
        // 更新蓝牙状态
        bt_update_status(eg_connected, 99, g_send_count_4g, g_send_count_bd, g_send_fail_count);

        printf("[MAIN] path=%d, 4g_avail=%d, eg_connected=%d\n", 
               send_path, current_4g_avail, eg_connected);

        // 从文件打包数据
        if (pack_data_from_files(paths, offsets, 2, EG_MSG_LEN,
                                 hex_buf, &hex_len, &entry_count) == 0)
        {
            if (entry_count > 0)
            {
                unsigned char raw_data[256];
                int raw_len = hex_to_bytes(hex_buf, raw_data, sizeof(raw_data));
                if (raw_len > 0)
                {
                    int send_ok = 0;
                    int tried_4g = 0;
                    int tried_bd = 0;

                    // 根据发送路径选择发送方式
                    switch (send_path)
                    {
                        case SEND_PATH_4G_ONLY:
                            // 仅4G发送
                            if (can_use_4g)
                            {
                                tried_4g = 1;
                                if (eg_send_data(raw_data, raw_len) == 0)
                                {
                                    send_ok = 1;
                                    g_send_count_4g++;
                                    printf("[MAIN] Sent %d bytes via 4G (4G_ONLY)\n", raw_len);
                                }
                            }
                            if (!send_ok)
                            {
                                printf("[MAIN] 4G_ONLY mode: 4G unavailable or send failed\n");
                                g_send_fail_count++;
                            }
                            break;

                        case SEND_PATH_BD_ONLY:
                            // 仅北斗发送
                            {
                                time_t now = time(NULL);
                                if (now - last_bd_send_time >= BD_SEND_INTERVAL)
                                {
                                    tried_bd = 1;
                                    if (bd_send_packet(raw_data, raw_len) == 0)
                                    {
                                        send_ok = 1;
                                        last_bd_send_time = now;
                                        g_send_count_bd++;
                                        printf("[MAIN] Sent %d bytes via BD (BD_ONLY)\n", raw_len);
                                    }
                                    else
                                    {
                                        printf("[MAIN] BD_ONLY mode: BD send failed\n");
                                        g_send_fail_count++;
                                    }
                                }
                                else
                                {
                                    printf("[MAIN] BD_ONLY mode: waiting for interval (%ld sec left)\n",
                                           BD_SEND_INTERVAL - (now - last_bd_send_time));
                                }
                            }
                            break;

                        case SEND_PATH_BD_FIRST:
                            // 北斗优先
                            {
                                time_t now = time(NULL);
                                if (now - last_bd_send_time >= BD_SEND_INTERVAL)
                                {
                                    tried_bd = 1;
                                    if (bd_send_packet(raw_data, raw_len) == 0)
                                    {
                                        send_ok = 1;
                                        last_bd_send_time = now;
                                        g_send_count_bd++;
                                        printf("[MAIN] Sent %d bytes via BD (BD_FIRST)\n", raw_len);
                                    }
                                }
                                // 北斗失败或间隔未到，尝试4G
                                if (!send_ok && can_use_4g)
                                {
                                    tried_4g = 1;
                                    if (eg_send_data(raw_data, raw_len) == 0)
                                    {
                                        send_ok = 1;
                                        g_send_count_4g++;
                                        printf("[MAIN] Sent %d bytes via 4G (BD_FIRST fallback)\n", raw_len);
                                    }
                                }
                            }
                            break;

                        case SEND_PATH_AUTO:
                        case SEND_PATH_4G_FIRST:
                        default:
                            // 自动或4G优先：先尝试4G，失败则北斗
                            if (can_use_4g)
                            {
                                tried_4g = 1;
                                if (eg_send_data(raw_data, raw_len) == 0)
                                {
                                    send_ok = 1;
                                    g_send_count_4g++;
                                    printf("[MAIN] Sent %d bytes via 4G\n", raw_len);
                                }
                                else
                                {
                                    // 4G发送失败，尝试重连
                                    printf("[MAIN] 4G send failed, trying reconnect...\n");
                                    eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
                                    if (eg_connect() == 0)
                                    {
                                        eg_connected = 1;
                                        if (eg_send_data(raw_data, raw_len) == 0)
                                        {
                                            send_ok = 1;
                                            g_send_count_4g++;
                                            printf("[MAIN] Sent %d bytes via 4G after reconnect\n", raw_len);
                                        }
                                        else
                                        {
                                            eg_connected = 0;
                                        }
                                    }
                                    else
                                    {
                                        eg_connected = 0;
                                    }
                                }
                            }

                            // 4G失败，尝试北斗
                            if (!send_ok)
                            {
                                time_t now = time(NULL);
                                if (now - last_bd_send_time >= BD_SEND_INTERVAL)
                                {
                                    tried_bd = 1;
                                    if (bd_send_packet(raw_data, raw_len) == 0)
                                    {
                                        send_ok = 1;
                                        last_bd_send_time = now;
                                        g_send_count_bd++;
                                        printf("[MAIN] Sent %d bytes via BD (fallback)\n", raw_len);
                                    }
                                    else
                                    {
                                        printf("[MAIN] BD send failed\n");
                                    }
                                }
                                else
                                {
                                    printf("[MAIN] BD interval not reached (%ld sec left)\n",
                                           BD_SEND_INTERVAL - (now - last_bd_send_time));
                                }
                            }
                            break;
                    }

                    if (send_ok)
                    {
                        save_offsets(OFFSET_FILE_MAIN, offsets, 2);
                    }
                    else
                    {
                        g_send_fail_count++;
                        printf("[MAIN] All channels failed (tried_4g=%d, tried_bd=%d)\n", 
                               tried_4g, tried_bd);
                    }
                }
            }
            else
            {
                // printf("[MAIN] No new data, sleeping...\n");
            }
        }
        else
        {
            printf("[MAIN] pack_data_from_files failed\n");
        }

        // 等待 10 秒再检查新数据
        for (int i = 0; i < 10 && !stop_flag; i++)
            sleep(1);
    }

    // ========== 3. 程序退出前清理 ==========
    if (eg_connected)
    {
        eg_send_cmd("AT+QICLOSE=0\r\n", "OK", 5);
    }
    eg_send_cmd("AT+QIDEACT=1\r\n", "OK", 40);
    save_offsets(OFFSET_FILE_MAIN, offsets, 2);
    return NULL;
}

void *watchdog_feed_thread(void *arg)
{
    while (!stop_flag)
    {
        if (watchdog_fd != -1)
        {
            if (write(watchdog_fd, "\0", 1) != 1)
            {
                perror("watchdog write");
            }
        }
        sleep(5); // 喂狗周期，需小于超时时间
    }
    return NULL;
}