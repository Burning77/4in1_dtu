/**
 * @file rtc.h
 * @brief RX8010SJ RTC 驱动接口
 * 
 * RX8010SJ 通过 I2C0 连接到 RK3566
 * 中断引脚: GPIO3_A5
 */

#ifndef __RTC_H__
#define __RTC_H__

#include <linux/rtc.h>
#include <stdint.h>

// ============== RX8010SJ I2C 地址 ==============
#define RX8010_I2C_ADDR     0x32    // 7位地址

// ============== RX8010SJ 寄存器地址 ==============
// 时间寄存器
#define RX8010_REG_SEC      0x10    // 秒 (BCD)
#define RX8010_REG_MIN      0x11    // 分 (BCD)
#define RX8010_REG_HOUR     0x12    // 时 (BCD)
#define RX8010_REG_WEEK     0x13    // 星期
#define RX8010_REG_DAY      0x14    // 日 (BCD)
#define RX8010_REG_MONTH    0x15    // 月 (BCD)
#define RX8010_REG_YEAR     0x16    // 年 (BCD, 00-99)

// 闹钟寄存器
#define RX8010_REG_ALMIN    0x17    // 闹钟分 (BCD)
#define RX8010_REG_ALHOUR   0x18    // 闹钟时 (BCD)
#define RX8010_REG_ALWDAY   0x19    // 闹钟星期/日

// 定时器寄存器
#define RX8010_REG_TCOUNT0  0x1A    // 定时器计数低字节
#define RX8010_REG_TCOUNT1  0x1B    // 定时器计数高字节

// 控制寄存器
#define RX8010_REG_EXT      0x1C    // 扩展寄存器
#define RX8010_REG_FLAG     0x1D    // 标志寄存器
#define RX8010_REG_CTRL     0x1E    // 控制寄存器

// 保留寄存器 (需要初始化)
#define RX8010_REG_RES1     0x1F    // 保留1 (写0xD8)
#define RX8010_REG_RES2     0x30    // 保留2 (写0x00)
#define RX8010_REG_RES3     0x31    // 保留3 (写0x08)
#define RX8010_REG_IRQ      0x32    // IRQ控制 (写0x00)

// ============== 寄存器位定义 ==============
// FLAG 寄存器 (0x1D)
#define RX8010_FLAG_VLF     (1 << 1)    // 电压低标志
#define RX8010_FLAG_AF      (1 << 3)    // 闹钟标志
#define RX8010_FLAG_TF      (1 << 4)    // 定时器标志
#define RX8010_FLAG_UF      (1 << 5)    // 更新标志

// CTRL 寄存器 (0x1E)
#define RX8010_CTRL_AIE     (1 << 3)    // 闹钟中断使能
#define RX8010_CTRL_TIE     (1 << 4)    // 定时器中断使能
#define RX8010_CTRL_UIE     (1 << 5)    // 更新中断使能
#define RX8010_CTRL_STOP    (1 << 6)    // 停止位
#define RX8010_CTRL_TEST    (1 << 7)    // 测试位

// EXT 寄存器 (0x1C)
#define RX8010_EXT_TSEL0    (1 << 0)    // 定时器时钟选择位0
#define RX8010_EXT_TSEL1    (1 << 1)    // 定时器时钟选择位1
#define RX8010_EXT_FSEL0    (1 << 2)    // FOUT频率选择位0
#define RX8010_EXT_FSEL1    (1 << 3)    // FOUT频率选择位1
#define RX8010_EXT_TE       (1 << 4)    // 定时器使能
#define RX8010_EXT_USEL     (1 << 5)    // 更新中断选择
#define RX8010_EXT_WADA     (1 << 6)    // 闹钟星期/日选择

// 闹钟寄存器 AE 位
#define RX8010_ALARM_AE     (1 << 7)    // 闹钟使能位 (0=使能, 1=禁用)

// ============== 定时器时钟源选择 ==============
#define RX8010_TSEL_4096HZ  0x00    // 4096 Hz
#define RX8010_TSEL_64HZ    0x01    // 64 Hz
#define RX8010_TSEL_1HZ     0x02    // 1 Hz (秒)
#define RX8010_TSEL_1_60HZ  0x03    // 1/60 Hz (分钟)

// ============== GPIO 中断引脚定义 ==============
// GPIO3_A5 = gpiochip3, line 5
#define RTC_IRQ_CHIP        "/dev/gpiochip3"
#define RTC_IRQ_LINE        5

// ============== 函数声明 ==============

/**
 * @brief 初始化 RX8010SJ RTC (通过 I2C)
 * @return 0=成功, -1=失败
 */
int rx8010_init(void);

/**
 * @brief 关闭 RX8010SJ
 */
void rx8010_close(void);

/**
 * @brief 读取 RTC 时间
 * @param tm 输出时间结构
 * @return 0=成功, -1=失败
 */
int rx8010_get_time(struct rtc_time *tm);

/**
 * @brief 设置 RTC 时间
 * @param tm 时间结构
 * @return 0=成功, -1=失败
 */
int rx8010_set_time(const struct rtc_time *tm);

/**
 * @brief 设置闹钟 (绝对时间)
 * @param hour 小时 (0-23)
 * @param min 分钟 (0-59)
 * @param wday 星期 (0-6, -1=每天)
 * @return 0=成功, -1=失败
 */
int rx8010_set_alarm(int hour, int min, int wday);

/**
 * @brief 设置定时器唤醒 (相对时间)
 * @param seconds 秒数 (1-65535)
 * @return 0=成功, -1=失败
 */
int rx8010_set_timer_wakeup(int seconds);

/**
 * @brief 清除闹钟/定时器中断标志
 * @return 0=成功, -1=失败
 */
int rx8010_clear_irq(void);

/**
 * @brief 禁用所有中断
 * @return 0=成功, -1=失败
 */
int rx8010_disable_irq(void);

/**
 * @brief 检查中断标志
 * @param alarm_flag 输出闹钟标志
 * @param timer_flag 输出定时器标志
 * @return 0=成功, -1=失败
 */
int rx8010_check_irq(int *alarm_flag, int *timer_flag);

/**
 * @brief 初始化 RTC 中断 GPIO
 * @return 0=成功, -1=失败
 */
int rtc_irq_init(void);

/**
 * @brief 等待 RTC 中断
 * @param timeout_ms 超时毫秒 (-1=永久等待)
 * @return 1=中断触发, 0=超时, -1=错误
 */
int rtc_wait_irq(int timeout_ms);

/**
 * @brief 释放 RTC 中断 GPIO
 */
void rtc_irq_cleanup(void);

/**
 * @brief RTC 唤醒线程函数
 * @param arg 唤醒间隔秒数 (int*)
 * @return NULL
 */
void *rtc_wakeup_thread(void *arg);

// ============== 兼容旧接口 ==============
int rtc_set_alarm(int rtc_fd, int seconds);
int rtc_set_time(struct rtc_time *rtc_tm);
int rtc_get_time(int fd, struct rtc_time *tm);
void rtc_clear_alarm(int rtc_fd);

#endif /* __RTC_H__ */
