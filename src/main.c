#include "../app/thread_summary.h"
struct kfifo data_fifo;
static unsigned char fifo_buffer[FIFO_SIZE];
volatile sig_atomic_t stop_flag = 0;
int rtc_fd;
extern int rs485_fd;
extern int rs232_fd;
extern int bd_fd;
extern int bt_fd;
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
    if (uart_init_gather())
    {
        gpio_cleanup();
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
        .tm_mday = 31,
        .tm_hour = 12,
        .tm_min = 0,
        .tm_sec = 0,
        .tm_isdst = 0};
    rtc_set_time(&set_time);
    rtc_fd = open("/dev/rtc0", O_RDONLY);
    loRa_Para_t my_lora_config = {
        .rf_freq = 433000000, // 中心频率：433 MHz
        .tx_power = 14,       // 发射功率：14 dBm
        .lora_sf = 7,         // 扩频因子：SF7 (需与 llcc68.h 中定义匹配，如 LORA_SF7)
        .band_width = 0x04,   // 带宽：125 kHz (对应 LORA_BW_125，值0x04)
        .code_rate = 0x01,    // 编码率：4/5 (对应 LORA_CR_4_5，值0x01)
        .payload_size = 64    // 预期接收负载的最大长度
    };
    printf("Initializing LoRa module...\n");
    if (!Lora_init(&my_lora_config))
    { // 将配置参数传递给驱动
        fprintf(stderr, "LoRa module initialization failed!\n");
        return -1;
    }
    printf("LoRa module initialized successfully.\n");
    // 创建线程
    pthread_t recv_tid, send_tid, rtc_tid, write_tid, bd_tid,lora_tid;
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
    if(pthread_create(&lora_tid, NULL, lora_transform_thread, NULL) != 0)
    {
        perror("pthread_create lora_thread");
        stop_flag = 1;
        pthread_join(recv_tid, NULL);
        pthread_join(send_tid, NULL);
        pthread_join(rtc_tid, NULL);
        pthread_join(write_tid, NULL);
        pthread_join(bd_tid, NULL);
        return 1;
    }
    // 等待线程结束
    pthread_join(recv_tid, NULL);
    pthread_join(send_tid, NULL);
    pthread_join(rtc_tid, NULL);
    pthread_join(write_tid, NULL);
    pthread_join(bd_tid, NULL);
    pthread_join(lora_tid, NULL);

    // 清理资源
    uart_close(rs232_fd);
    uart_close(rs485_fd);
    uart_close(bd_fd);
    gpio_cleanup();
    close(rtc_fd);

    printf("Program terminated.\n");
    return 0;
}