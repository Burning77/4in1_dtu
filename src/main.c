#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/select.h>
#include <errno.h>
#include "../inc/gpio.h"
#include "../inc/usart.h"
#include "../inc/rtc.h"
#include <linux/rtc.h>
#include <fcntl.h>
#include "../inc/universal.h"
#include "../inc/kfifo.h"
#include <time.h>

static off_t last_sent_offset_485 = 0;
static off_t last_sent_offset_232 = 0;

static struct kfifo data_fifo;
static unsigned char fifo_buffer[FIFO_SIZE];
static pthread_mutex_t fifo_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t fifo_not_empty = PTHREAD_COND_INITIALIZER;
static volatile sig_atomic_t stop_flag = 0;
int rtc_fd;
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
static int rs485_fd = -1;
static int rs232_fd = -1;
static int bd_fd = -1;
// static int rtc_fd;
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
    serial_state_t rs485_state, rs232_state, bd_state;
    serial_state_init(&rs485_state, RS485_DATA, "RS485");
    serial_state_init(&rs232_state, RS232_DATA, "RS232");
    serial_state_init(&bd_state, BD_DATA, "BD");

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
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms 超时
        max_fd = MAX(MAX(rs485_fd, rs232_fd), bd_fd);

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
    }

    // 退出前处理残留帧
    flush_serial_state(&rs485_state, &ctx);
    flush_serial_state(&rs232_state, &ctx);
    flush_serial_state(&bd_state, &ctx);

    return NULL;
}

// 发送线程
void *sensor_send_thread(void *arg)
{
    const char *test = "Hello from rs232!\r\n";
    while (!stop_flag)
    {
        sleep(SEND_INTERVAL);
        if (stop_flag)
            break;

        printf("[SEND] Sending command...\n");
        int resualt = data_send(test, strlen(test), RS232_DEV);
        int ret = data_send(CMD_STRING, strlen(CMD_STRING), RS485_DEV);
        int res_bd = data_send(BD_CARD, strlen(BD_CARD), BD_DEV);
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
    FILE *fp_485 = NULL, *fp_232 = NULL;
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

        // 选择文件
        FILE **fp = (msg.type == RS485_DATA) ? &fp_485 : &fp_232;
        const char *path = (msg.type == RS485_DATA) ? RS485_LOG_PATH : RS232_LOG_PATH;

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
    return NULL;
}
void *bd_send_thread(void *arg)
{
    char line[512];
    char hex_buf[BD_MSG_LEN + 64];
    char msg[512];
    const char *paths[] = {RS485_LOG_PATH, RS232_LOG_PATH};
    off_t *offsets[] = {&last_sent_offset_485, &last_sent_offset_232};
    int i, j;

    // 等待日志文件被创建（最多等待30秒）
    while (!stop_flag)
    {
        hex_buf[0] = '\0';
        int hex_len = 0;
        int entry_count = 0;
        int has_valid_data = 0; // 新增：标记本次循环是否有有效数据

        // 首先检查两个文件是否存在
        int files_exist = 0;
        for (i = 0; i < 2; i++)
        {
            if (access(paths[i], F_OK) == 0)
            {
                files_exist = 1;
                break;
            }
        }

        // 如果两个文件都不存在，直接等待，不发送任何报文
        if (!files_exist)
        {
            sleep(5);
            continue; // 跳过本次循环，进入下一次循环
        }

        // 依次处理两个文件
        for (i = 0; i < 2; i++)
        {
            FILE *fp = fopen(paths[i], "r");
            if (!fp)
            {
                // 文件可能在上一次检查后被删除，跳过
                continue;
            }

            // 定位到上次发送位置
            fseeko(fp, *offsets[i], SEEK_SET);

            while (fgets(line, sizeof(line), fp))
            {
                unsigned char raw[256];
                int raw_len = parse_log_line(line, raw, sizeof(raw));
                if (raw_len < 2)
                    continue; // 跳过无效行

                int need_hex = raw_len * 2; // 需要的十六进制字符数
                int total_need = hex_len + (entry_count > 0 ? 1 : 0) + need_hex;

                if (total_need > BD_MSG_LEN)
                {
                    // 空间不足，回退一行，留待下次处理
                    fseeko(fp, -(off_t)strlen(line), SEEK_CUR);
                    break;
                }

                // 有有效数据，设置标志
                has_valid_data = 1;

                // 添加逗号分隔（除了第一个条目）
                if (entry_count > 0)
                {
                    strcat(hex_buf, ",");
                    hex_len++;
                }

                // 将原始数据转换为大写十六进制追加到 hex_buf
                char hex_part[512];
                char *p = hex_part;
                for (j = 0; j < raw_len; j++)
                {
                    p += sprintf(p, "%02X", raw[j]);
                }
                strcat(hex_buf, hex_part);
                hex_len += need_hex;
                entry_count++;

                // 更新偏移量
                *offsets[i] = ftello(fp);
            }

            fclose(fp);
        }

        // 构造并发送报文
        if (has_valid_data && entry_count > 0)
        {
            snprintf(msg, sizeof(msg), "$CCTCQ,4314513,2,1,2,,%s*", hex_buf);
            unsigned char cs = calc_checksum(msg);
            size_t len = strlen(msg);
            snprintf(msg + len, sizeof(msg) - len, "%02X\r\n", cs);
            data_send((unsigned char *)msg, strlen(msg), BD_DEV);
            printf("[SEND BDTX] %s", msg);
        }
        else
        {
            // 文件存在但没有有效数据，不发送任何报文
            printf("[INFO] No valid data in log files, skipping BD transmission.\n");
        }

        // 等待60秒
        for (i = 0; i < 60 && !stop_flag; i++)
        {
            sleep(1);
        }
    }
    return NULL;
}
int main(int argc, char *argv[])
{
    // 注册信号处理
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    kfifo_init(&data_fifo, fifo_buffer, FIFO_SIZE);
    // 初始化GPIO
    if (gpio_init() < 0)
    {
        return 1;
    }
    // 初始化rs485
    rs485_fd = uart_init(RS485_DEV, RS485_BAUD);
    if (rs485_fd < 0)
    {
        gpio_cleanup();
        return 1;
    }
    // 初始化rs232
    rs232_fd = uart_init(RS232_DEV, RS232_BAUD);
    if (rs232_fd < 0)
    {
        return 1;
    }
    bd_fd = uart_init(BD_DEV, BD_BAUD);
    if (bd_fd < 0)
    {
        return 1;
    }
    rf_power_on();
    //  sleep(10);

    // // 发送读卡号指令
    // char cmd[] = "$CCICR,0,00 * 68\r\n";
    // write(bd_fd, cmd, strlen(cmd));

    // // 等待并读取回复
    // sleep(2);
    // char buf[256];
    // int n = read(bd_fd, buf, sizeof(buf));
    // printf("Received %d bytes\n", n);
    // for (int i = 0; i < n; i++) {
    //     printf("%c", buf[i]);
    // }
    struct rtc_time set_time = {
        .tm_year = 2026 - 1900,
        .tm_mon = 3 - 1, // 3月
        .tm_mday = 6,
        .tm_hour = 12,
        .tm_min = 0,
        .tm_sec = 0,
        .tm_isdst = 0};
    rtc_set_time(&set_time);
    rtc_fd = open("/dev/rtc0", O_RDONLY);
    // 创建线程
    pthread_t recv_tid, send_tid, rtc_tid, write_tid, bd_tid;
    if (pthread_create(&recv_tid, NULL, receive_thread, NULL) != 0)
    {
        perror("pthread_create recv");
        uart_close(rs485_fd);
        uart_close(rs232_fd);
        gpio_cleanup();
        return 1;
    }
    if (pthread_create(&send_tid, NULL, sensor_send_thread, NULL) != 0)
    {
        perror("pthread_create send");
        stop_flag = 1;
        pthread_join(recv_tid, NULL);
        uart_close(rs485_fd);
        uart_close(rs232_fd);
        gpio_cleanup();
        return 1;
    }
    if (pthread_create(&rtc_tid, NULL, read_rtc_thread, NULL) != 0)
    {
        perror("pthread_create rtc");
        stop_flag = 1;
        pthread_join(recv_tid, NULL);
        pthread_join(send_tid, NULL);
        return 1;
    }
    if (pthread_create(&write_tid, NULL, write_file_thread, NULL) != 0)
    {
        perror("pthread_create write");
        stop_flag = 1;
        pthread_join(recv_tid, NULL);
        pthread_join(send_tid, NULL);
        pthread_join(rtc_tid, NULL);
        return 1;
    }
    if (pthread_create(&bd_tid, NULL, bd_send_thread, NULL) != 0)
    {
        perror("pthread_create bd_thread");
        stop_flag = 1;
        pthread_join(recv_tid, NULL);
        pthread_join(send_tid, NULL);
        pthread_join(rtc_tid, NULL);
        pthread_join(write_tid, NULL);
        return 1;
    }
    // 等待线程结束
    pthread_join(recv_tid, NULL);
    pthread_join(send_tid, NULL);
    pthread_join(rtc_tid, NULL);
    pthread_join(write_tid, NULL);
    pthread_join(bd_tid, NULL);

    // 清理资源
    uart_close(rs232_fd);
    uart_close(rs485_fd);
    uart_close(bd_fd);
    gpio_cleanup();
    close(rtc_fd);

    printf("Program terminated.\n");
    return 0;
}