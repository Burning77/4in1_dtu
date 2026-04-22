/**
 * @file rtc.c
 * @brief RX8010SJ RTC 驱动实现
 * 
 * 通过 I2C 直接操作 RX8010SJ 寄存器实现:
 * - 时间读写
 * - 闹钟设置
 * - 定时器唤醒
 * - 中断处理
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/rtc.h>
#include <gpiod.h>
#include <time.h>
#include <poll.h>
#include <stdatomic.h>
#include "../inc/rtc.h"

// ============== 全局变量 ==============
static int i2c_fd = -1;
static struct gpiod_chip *rtc_irq_chip = NULL;
static struct gpiod_line *rtc_irq_line = NULL;
extern atomic_int stop_flag;

// ============== BCD 转换函数 ==============
static inline uint8_t bcd2bin(uint8_t bcd)
{
    return (bcd & 0x0F) + ((bcd >> 4) * 10);
}

static inline uint8_t bin2bcd(uint8_t bin)
{
    return ((bin / 10) << 4) | (bin % 10);
}

// ============== I2C 操作函数 ==============

/**
 * @brief 写入单个寄存器
 */
static int rx8010_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    if (write(i2c_fd, buf, 2) != 2) {
        perror("[RTC] I2C write failed");
        return -1;
    }
    return 0;
}

/**
 * @brief 读取单个寄存器
 */
static int rx8010_read_reg(uint8_t reg, uint8_t *value)
{
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("[RTC] I2C write reg addr failed");
        return -1;
    }
    if (read(i2c_fd, value, 1) != 1) {
        perror("[RTC] I2C read failed");
        return -1;
    }
    return 0;
}

/**
 * @brief 读取多个连续寄存器
 */
static int rx8010_read_regs(uint8_t reg, uint8_t *buf, int len)
{
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("[RTC] I2C write reg addr failed");
        return -1;
    }
    if (read(i2c_fd, buf, len) != len) {
        perror("[RTC] I2C read multiple failed");
        return -1;
    }
    return 0;
}

/**
 * @brief 写入多个连续寄存器
 */
static int rx8010_write_regs(uint8_t reg, const uint8_t *buf, int len)
{
    uint8_t *data = malloc(len + 1);
    if (!data) return -1;
    
    data[0] = reg;
    memcpy(data + 1, buf, len);
    
    int ret = (write(i2c_fd, data, len + 1) == len + 1) ? 0 : -1;
    free(data);
    
    if (ret < 0) {
        perror("[RTC] I2C write multiple failed");
    }
    return ret;
}

// ============== RX8010SJ 初始化 ==============

int rx8010_init(void)
{
    // 打开 I2C 设备
    i2c_fd = open("/dev/i2c-0", O_RDWR);
    if (i2c_fd < 0) {
        perror("[RTC] Failed to open /dev/i2c-0");
        return -1;
    }
    
    // 设置 I2C 从设备地址
    if (ioctl(i2c_fd, I2C_SLAVE, RX8010_I2C_ADDR) < 0) {
        perror("[RTC] Failed to set I2C slave address");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }
    
    // 初始化保留寄存器 (RX8010SJ 数据手册要求)
    rx8010_write_reg(RX8010_REG_RES1, 0xD8);
    rx8010_write_reg(RX8010_REG_RES2, 0x00);
    rx8010_write_reg(RX8010_REG_RES3, 0x08);
    rx8010_write_reg(RX8010_REG_IRQ, 0x00);
    
    // 检查 VLF 标志 (电压低)
    uint8_t flag;
    if (rx8010_read_reg(RX8010_REG_FLAG, &flag) == 0) {
        if (flag & RX8010_FLAG_VLF) {
            printf("[RTC] Warning: Voltage low flag set, time may be invalid\n");
            // 清除 VLF 标志
            rx8010_write_reg(RX8010_REG_FLAG, flag & ~RX8010_FLAG_VLF);
        }
    }
    
    // 清除所有中断标志
    rx8010_clear_irq();
    
    printf("[RTC] RX8010SJ initialized on I2C-0\n");
    return 0;
}

void rx8010_close(void)
{
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

// ============== 时间操作 ==============

int rx8010_get_time(struct rtc_time *tm)
{
    if (i2c_fd < 0) return -1;
    
    uint8_t buf[7];
    if (rx8010_read_regs(RX8010_REG_SEC, buf, 7) < 0) {
        return -1;
    }
    
    tm->tm_sec = bcd2bin(buf[0] & 0x7F);
    tm->tm_min = bcd2bin(buf[1] & 0x7F);
    tm->tm_hour = bcd2bin(buf[2] & 0x3F);  // 24小时制
    tm->tm_wday = ffs(buf[3]) - 1;          // 星期 (0-6)
    tm->tm_mday = bcd2bin(buf[4] & 0x3F);
    tm->tm_mon = bcd2bin(buf[5] & 0x1F) - 1; // 月份 (0-11)
    tm->tm_year = bcd2bin(buf[6]) + 100;     // 年份 (从1900开始)
    
    return 0;
}

int rx8010_set_time(const struct rtc_time *tm)
{
    if (i2c_fd < 0) return -1;
    
    uint8_t buf[7];
    buf[0] = bin2bcd(tm->tm_sec);
    buf[1] = bin2bcd(tm->tm_min);
    buf[2] = bin2bcd(tm->tm_hour);
    buf[3] = 1 << tm->tm_wday;              // 星期位图
    buf[4] = bin2bcd(tm->tm_mday);
    buf[5] = bin2bcd(tm->tm_mon + 1);
    buf[6] = bin2bcd(tm->tm_year - 100);
    
    // 停止 RTC
    uint8_t ctrl;
    rx8010_read_reg(RX8010_REG_CTRL, &ctrl);
    rx8010_write_reg(RX8010_REG_CTRL, ctrl | RX8010_CTRL_STOP);
    
    // 写入时间
    int ret = rx8010_write_regs(RX8010_REG_SEC, buf, 7);
    
    // 启动 RTC
    rx8010_write_reg(RX8010_REG_CTRL, ctrl & ~RX8010_CTRL_STOP);
    
    if (ret == 0) {
        printf("[RTC] Time set to %04d-%02d-%02d %02d:%02d:%02d\n",
               tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
               tm->tm_hour, tm->tm_min, tm->tm_sec);
    }
    
    return ret;
}

// ============== 闹钟功能 ==============

int rx8010_set_alarm(int hour, int min, int wday)
{
    if (i2c_fd < 0) return -1;
    
    // 禁用闹钟中断
    uint8_t ctrl;
    rx8010_read_reg(RX8010_REG_CTRL, &ctrl);
    rx8010_write_reg(RX8010_REG_CTRL, ctrl & ~RX8010_CTRL_AIE);
    
    // 清除闹钟标志
    uint8_t flag;
    rx8010_read_reg(RX8010_REG_FLAG, &flag);
    rx8010_write_reg(RX8010_REG_FLAG, flag & ~RX8010_FLAG_AF);
    
    // 设置闹钟时间
    uint8_t al_min = bin2bcd(min);           // 分钟使能
    uint8_t al_hour = bin2bcd(hour);         // 小时使能
    uint8_t al_wday;
    
    if (wday >= 0 && wday <= 6) {
        al_wday = 1 << wday;                 // 指定星期
    } else {
        al_wday = RX8010_ALARM_AE;           // 每天 (AE=1 禁用星期匹配)
    }
    
    rx8010_write_reg(RX8010_REG_ALMIN, al_min);
    rx8010_write_reg(RX8010_REG_ALHOUR, al_hour);
    rx8010_write_reg(RX8010_REG_ALWDAY, al_wday);
    
    // 设置 WADA 位选择星期/日模式
    uint8_t ext;
    rx8010_read_reg(RX8010_REG_EXT, &ext);
    ext &= ~RX8010_EXT_WADA;  // 0 = 星期模式
    rx8010_write_reg(RX8010_REG_EXT, ext);
    
    // 启用闹钟中断
    rx8010_write_reg(RX8010_REG_CTRL, ctrl | RX8010_CTRL_AIE);
    
    printf("[RTC] Alarm set for %02d:%02d (wday=%d)\n", hour, min, wday);
    return 0;
}

// ============== 定时器唤醒功能 ==============

int rx8010_set_timer_wakeup(int seconds)
{
    if (i2c_fd < 0) return -1;
    if (seconds < 1 || seconds > 65535) {
        printf("[RTC] Invalid timer value: %d (must be 1-65535)\n", seconds);
        return -1;
    }
    
    // 禁用定时器
    uint8_t ext;
    rx8010_read_reg(RX8010_REG_EXT, &ext);
    rx8010_write_reg(RX8010_REG_EXT, ext & ~RX8010_EXT_TE);
    
    // 禁用定时器中断
    uint8_t ctrl;
    rx8010_read_reg(RX8010_REG_CTRL, &ctrl);
    rx8010_write_reg(RX8010_REG_CTRL, ctrl & ~RX8010_CTRL_TIE);
    
    // 清除定时器标志
    uint8_t flag;
    rx8010_read_reg(RX8010_REG_FLAG, &flag);
    rx8010_write_reg(RX8010_REG_FLAG, flag & ~RX8010_FLAG_TF);
    
    // 设置定时器时钟源为 1Hz (秒)
    ext &= ~(RX8010_EXT_TSEL0 | RX8010_EXT_TSEL1);
    ext |= RX8010_TSEL_1HZ;
    rx8010_write_reg(RX8010_REG_EXT, ext);
    
    // 设置定时器计数值 (16位)
    rx8010_write_reg(RX8010_REG_TCOUNT0, seconds & 0xFF);
    rx8010_write_reg(RX8010_REG_TCOUNT1, (seconds >> 8) & 0xFF);
    
    // 启用定时器中断
    rx8010_write_reg(RX8010_REG_CTRL, ctrl | RX8010_CTRL_TIE);
    
    // 启用定时器
    rx8010_read_reg(RX8010_REG_EXT, &ext);
    rx8010_write_reg(RX8010_REG_EXT, ext | RX8010_EXT_TE);
    
    printf("[RTC] Timer wakeup set for %d seconds\n", seconds);
    return 0;
}

// ============== 中断处理 ==============

int rx8010_clear_irq(void)
{
    if (i2c_fd < 0) return -1;
    
    uint8_t flag;
    if (rx8010_read_reg(RX8010_REG_FLAG, &flag) < 0) {
        return -1;
    }
    
    // 清除 AF 和 TF 标志
    flag &= ~(RX8010_FLAG_AF | RX8010_FLAG_TF | RX8010_FLAG_UF);
    return rx8010_write_reg(RX8010_REG_FLAG, flag);
}

int rx8010_disable_irq(void)
{
    if (i2c_fd < 0) return -1;
    
    // 禁用所有中断
    uint8_t ctrl;
    rx8010_read_reg(RX8010_REG_CTRL, &ctrl);
    ctrl &= ~(RX8010_CTRL_AIE | RX8010_CTRL_TIE | RX8010_CTRL_UIE);
    rx8010_write_reg(RX8010_REG_CTRL, ctrl);
    
    // 禁用定时器
    uint8_t ext;
    rx8010_read_reg(RX8010_REG_EXT, &ext);
    rx8010_write_reg(RX8010_REG_EXT, ext & ~RX8010_EXT_TE);
    
    // 清除中断标志
    return rx8010_clear_irq();
}

int rx8010_check_irq(int *alarm_flag, int *timer_flag)
{
    if (i2c_fd < 0) return -1;
    
    uint8_t flag;
    if (rx8010_read_reg(RX8010_REG_FLAG, &flag) < 0) {
        return -1;
    }
    
    if (alarm_flag) *alarm_flag = (flag & RX8010_FLAG_AF) ? 1 : 0;
    if (timer_flag) *timer_flag = (flag & RX8010_FLAG_TF) ? 1 : 0;
    
    return 0;
}

// ============== GPIO 中断 ==============

int rtc_irq_init(void)
{
    rtc_irq_chip = gpiod_chip_open(RTC_IRQ_CHIP);
    if (!rtc_irq_chip) {
        perror("[RTC] Failed to open GPIO chip for IRQ");
        return -1;
    }
    
    rtc_irq_line = gpiod_chip_get_line(rtc_irq_chip, RTC_IRQ_LINE);
    if (!rtc_irq_line) {
        perror("[RTC] Failed to get GPIO line for IRQ");
        gpiod_chip_close(rtc_irq_chip);
        rtc_irq_chip = NULL;
        return -1;
    }
    
    // 请求为下降沿中断输入 (RX8010SJ IRQ 为低电平有效)
    if (gpiod_line_request_falling_edge_events(rtc_irq_line, "rtc_irq") < 0) {
        perror("[RTC] Failed to request GPIO IRQ events");
        gpiod_chip_close(rtc_irq_chip);
        rtc_irq_chip = NULL;
        rtc_irq_line = NULL;
        return -1;
    }
    
    printf("[RTC] IRQ GPIO initialized (GPIO3_A5)\n");
    return 0;
}

int rtc_wait_irq(int timeout_ms)
{
    if (!rtc_irq_line) return -1;
    
    struct timespec ts;
    if (timeout_ms < 0) {
        ts.tv_sec = 0;
        ts.tv_nsec = 0;
    } else {
        ts.tv_sec = timeout_ms / 1000;
        ts.tv_nsec = (timeout_ms % 1000) * 1000000;
    }
    
    int ret = gpiod_line_event_wait(rtc_irq_line, timeout_ms < 0 ? NULL : &ts);
    if (ret == 1) {
        // 读取并清除事件
        struct gpiod_line_event event;
        gpiod_line_event_read(rtc_irq_line, &event);
        return 1;  // 中断触发
    } else if (ret == 0) {
        return 0;  // 超时
    }
    return -1;  // 错误
}

void rtc_irq_cleanup(void)
{
    if (rtc_irq_line) {
        gpiod_line_release(rtc_irq_line);
        rtc_irq_line = NULL;
    }
    if (rtc_irq_chip) {
        gpiod_chip_close(rtc_irq_chip);
        rtc_irq_chip = NULL;
    }
}

// ============== RTC 唤醒线程 ==============

void *rtc_wakeup_thread(void *arg)
{
    int wakeup_interval = arg ? *(int *)arg : 60;  // 默认60秒
    
    printf("[RTC] Wakeup thread started, interval=%d seconds\n", wakeup_interval);
    
    // 初始化 RTC
    if (rx8010_init() < 0) {
        printf("[RTC] Failed to initialize RX8010SJ\n");
        return NULL;
    }
    
    // 初始化 IRQ GPIO
    if (rtc_irq_init() < 0) {
        printf("[RTC] Failed to initialize IRQ GPIO\n");
        rx8010_close();
        return NULL;
    }
    
    while (!stop_flag) {
        // 设置定时器唤醒
        if (rx8010_set_timer_wakeup(wakeup_interval) < 0) {
            printf("[RTC] Failed to set timer wakeup\n");
            sleep(wakeup_interval);
            continue;
        }
        
        // 等待中断 (超时时间比唤醒间隔多10秒)
        int ret = rtc_wait_irq((wakeup_interval + 10) * 1000);
        
        if (ret == 1) {
            // 中断触发
            int alarm_flag, timer_flag;
            rx8010_check_irq(&alarm_flag, &timer_flag);
            
            if (timer_flag) {
                printf("[RTC] Timer wakeup triggered!\n");
            }
            if (alarm_flag) {
                printf("[RTC] Alarm triggered!\n");
            }
            
            // 清除中断标志
            rx8010_clear_irq();
            
            // 这里可以添加唤醒后的处理逻辑
            // 例如: 触发数据发送、检查传感器等
            
        } else if (ret == 0) {
            printf("[RTC] Wait timeout (no IRQ)\n");
        } else {
            printf("[RTC] Wait IRQ error\n");
        }
    }
    
    // 清理
    rx8010_disable_irq();
    rtc_irq_cleanup();
    rx8010_close();
    
    printf("[RTC] Wakeup thread stopped\n");
    return NULL;
}

// ============== 兼容旧接口 ==============

int rtc_set_alarm(int rtc_fd, int seconds)
{
    (void)rtc_fd;  // 不再使用 /dev/rtc0
    
    // 计算目标时间
    struct rtc_time now;
    if (rx8010_get_time(&now) < 0) {
        return -1;
    }
    
    struct tm tm_now = {
        .tm_sec = now.tm_sec,
        .tm_min = now.tm_min,
        .tm_hour = now.tm_hour,
        .tm_mday = now.tm_mday,
        .tm_mon = now.tm_mon,
        .tm_year = now.tm_year,
        .tm_isdst = -1
    };
    
    time_t t = mktime(&tm_now) + seconds;
    struct tm *tm_alarm = localtime(&t);
    
    return rx8010_set_alarm(tm_alarm->tm_hour, tm_alarm->tm_min, -1);
}

int rtc_set_time(struct rtc_time *rtc_tm)
{
    return rx8010_set_time(rtc_tm);
}

int rtc_get_time(int fd, struct rtc_time *tm)
{
    (void)fd;  // 不再使用 /dev/rtc0
    return rx8010_get_time(tm);
}

void rtc_clear_alarm(int rtc_fd)
{
    (void)rtc_fd;
    rx8010_disable_irq();
}
