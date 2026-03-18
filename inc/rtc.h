#ifndef USART_H
#define USART_H
#include <linux/rtc.h>
// 时间寄存器地址
#define SEC_ADDR     0x00  // 秒
#define MIN_ADDR     0x01  // 分
#define HOUR_ADDR    0x02  // 时
#define DAY_ADDR     0x03  // 日
#define WEEK_ADDR    0x04  // 星期
#define MONTH_ADDR   0x05  // 月
#define YEAR_ADDR    0x06  // 年

// 用户RAM地址（用于存储数据，掉电不丢）
#define USER_RAM_START  0x40
#define USER_RAM_SIZE   16   // 128位 = 16字节

int rtc_set_time(struct rtc_time *rtc_tm);
int rtc_get_time(int fd, struct rtc_time *tm);
#endif