#include "../app/thread_summary.h"
#include "../inc/watch_dog.h"
#include "../inc/bluetooth.h"
#include "../inc/rtc.h"
#include "../inc/power.h"
#include "../inc/eg800k.h"
#include <stdatomic.h>
#include <getopt.h>

// 运行模式
#define MODE_NORMAL     0   // 正常模式 (多线程持续运行)
#define MODE_LOW_POWER  1   // 低功耗模式 (周期性休眠唤醒)

typedef void *(*thread_func_t)(void *);
atomic_int stop_flag = 0;
struct kfifo data_fifo;
static unsigned char fifo_buffer[FIFO_SIZE];

// 运行模式配置
static int run_mode = MODE_NORMAL;
static int rtc_wakeup_interval = 60;

// 低功耗模式配置
static power_config_t power_config = {
    .wakeup_interval = 10,          // 10秒唤醒一次
    .enable_4g_sleep = 1,
    .enable_bt_sleep = 1,
    .enable_lora_sleep = 1,
    .check_data_before_sleep = 1
};

int rtc_fd;
extern int rs485_fd;
extern int rs232_fd;
extern int bd_fd;
extern int bt_fd;
extern int eg_fd;
extern int watchdog_fd;

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("Options:\n");
    printf("  -m, --mode <mode>      Run mode: normal, lowpower (default: normal)\n");
    printf("  -i, --interval <sec>   Wakeup interval in seconds (default: 10)\n");
    printf("  -h, --help             Show this help\n");
    printf("\nExamples:\n");
    printf("  %s                     # Normal mode\n", prog);
    printf("  %s -m lowpower -i 10   # Low power mode, wake every 10 seconds\n", prog);
}
// int check_hardware_flow_control(int fd)
// {
//     struct termios options;
//     if (tcgetattr(fd, &options) != 0)
//     {
//         perror("tcgetattr");
//         return -1;
//     }

//     if (options.c_cflag & CRTSCTS)
//     {
//         printf("Hardware flow control (CRTSCTS) is ENABLED.\n");
//         return 1;
//     }
//     else
//     {
//         printf("Hardware flow control (CRTSCTS) is DISABLED.\n");
//         return 0;
//     }
// }
int main(int argc, char *argv[])
{
    // 关闭 stdout/stderr 缓冲，printf/perror 实时写入 journal
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // 解析命令行参数
    static struct option long_options[] = {
        {"mode",     required_argument, 0, 'm'},
        {"interval", required_argument, 0, 'i'},
        {"help",     no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "m:i:h", long_options, NULL)) != -1) {
        switch (opt) {
            case 'm':
                if (strcmp(optarg, "lowpower") == 0 || strcmp(optarg, "lp") == 0) {
                    run_mode = MODE_LOW_POWER;
                } else if (strcmp(optarg, "normal") == 0) {
                    run_mode = MODE_NORMAL;
                } else {
                    fprintf(stderr, "Unknown mode: %s\n", optarg);
                    print_usage(argv[0]);
                    return 1;
                }
                break;
            case 'i':
                power_config.wakeup_interval = atoi(optarg);
                rtc_wakeup_interval = power_config.wakeup_interval;
                if (power_config.wakeup_interval < 1) {
                    power_config.wakeup_interval = 10;
                }
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    printf("========================================\n");
    printf("  DTU Application Starting\n");
    printf("  Mode: %s\n", run_mode == MODE_LOW_POWER ? "LOW POWER" : "NORMAL");
    if (run_mode == MODE_LOW_POWER) {
        printf("  Wakeup interval: %d seconds\n", power_config.wakeup_interval);
    }
    printf("========================================\n");

    // 注册信号处理
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    kfifo_init(&data_fifo, fifo_buffer, FIFO_SIZE);
    // 初始化GPIO
    if (gpio_init() < 0)
    {
        return 1;
    }
    rf_power_on();

    // 等待 4G 模块 USB 枚举完成（EG800K 上电后需要较长时间）
    printf("Waiting for 4G module USB enumeration...\n");
    int uart_retry = 0;
    const int max_uart_retry = 10;
    while (uart_init_gather() != 0)
    {
        uart_retry++;
        if (uart_retry >= max_uart_retry)
        {
            fprintf(stderr, "UART init failed after %d retries, exiting\n", max_uart_retry);
            gpio_cleanup();
            return 1;
        }
        printf("UART init failed, retry %d/%d in 2 seconds...\n", uart_retry, max_uart_retry);
        sleep(2);
    }
    printf("All UARTs initialized successfully.\n");
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
    // printf("flow check\n");
    // check_hardware_flow_control(eg_fd);
    // printf("check over\r\n");
    // eg_init();
    // char cmd[] = "AT+CEREG?\r\n";
    // printf("write cmd to eg\r\n");
    // write(eg_fd, cmd, strlen(cmd));
    // printf("write cmd to eg over\r\n");
    // sleep(2);
    // char buf[256];
    // printf("read data from eg\r\n");
    // int n = read(eg_fd, buf, sizeof(buf));
    // printf("Received %s bytes\n", buf);
    struct rtc_time set_time = {
        .tm_year = 2026 - 1900,
        .tm_mon = 4 - 1, // 4月
        .tm_mday = 22,
        .tm_hour = 17,
        .tm_min = 0,
        .tm_sec = 0,
        .tm_isdst = 0};
    
    // 初始化 RX8010SJ RTC (通过 I2C)
    // 低功耗模式下不保持 RTC 打开，让 rtcwake 可以使用
    if (run_mode == MODE_NORMAL) {
        printf("Initializing RX8010SJ RTC...\n");
        if (rx8010_init() == 0) {
            rx8010_set_time(&set_time);
            printf("RX8010SJ RTC initialized successfully.\n");
        } else {
            printf("Warning: RX8010SJ RTC init failed, using system RTC\n");
            rtc_set_time(&set_time);
        }
        // 保留 /dev/rtc0 用于兼容 (仅正常模式)
        rtc_fd = open("/dev/rtc0", O_RDONLY);
    } else {
        printf("Low power mode: RTC will be managed by rtcwake\n");
        rtc_fd = -1;
    }
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
    
    // 根据运行模式选择线程配置
    int ret = 1;
    
    if (run_mode == MODE_LOW_POWER) {
        // ========== 低功耗模式 ==========
        // 只启动必要的线程，主循环由 power_manager_thread 控制
        printf("[MAIN] Starting in LOW POWER mode...\n");
        
        // 先初始化 4G 模块
        printf("[MAIN] Initializing 4G module...\n");
        if (eg_init() == 0) {
            printf("[MAIN] 4G module initialized\n");
            if (eg_connect() == 0) {
                printf("[MAIN] TCP connected\n");
            }
        }
        
        // 启动低功耗管理线程
        pthread_t power_tid;
        if (pthread_create(&power_tid, NULL, power_manager_thread, &power_config) != 0) {
            perror("pthread_create power_manager failed");
            goto cleanup;
        }
        
        // 等待线程结束
        pthread_join(power_tid, NULL);
        ret = 0;
        
    } else {
        // ========== 正常模式 ==========
        #define THREAD_COUNT_NORMAL 10
        thread_func_t thread_funcs[THREAD_COUNT_NORMAL] = {
            receive_thread,
            serial_send_thread,
            read_rtc_thread,
            write_file_thread,
            main_send_thread,
            lora_transform_thread,
            eg_monitor_thread,
            watchdog_feed_thread,
            bt_comm_thread,
            rtc_wakeup_thread
        };
        void *thread_args[THREAD_COUNT_NORMAL] = {
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
            &rtc_wakeup_interval
        };
        pthread_t tids[THREAD_COUNT_NORMAL];
        int created_count = 0;
        
        for (int i = 0; i < THREAD_COUNT_NORMAL; i++)
        {
            if (pthread_create(&tids[i], NULL, thread_funcs[i], thread_args[i]) != 0)
            {
                perror("pthread_create failed");
                atomic_store(&stop_flag, 1);
                goto cleanup;
            }
            created_count++;
        }
        
        for (int i = 0; i < THREAD_COUNT_NORMAL; i++)
        {
            pthread_join(tids[i], NULL);
        }
        ret = 0;
    }

cleanup:
    printf("[MAIN] Cleaning up...\n");
    rf_power_off();
    uart_close(rs232_fd);
    uart_close(rs485_fd);
    uart_close(bd_fd);
    uart_close(eg_fd);
    gpio_cleanup();
    close(rtc_fd);
    rx8010_close();
    cleanup_watchdog();
    printf("[MAIN] Exiting with code %d\n", ret);
    return ret;
}