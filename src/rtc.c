#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/rtc.h>
#include "../inc/rtc.h"
#include <time.h>
/**
 * 设置 RTC 闹钟，使系统在 seconds 秒后唤醒
 * @param rtc_fd  RTC 设备文件描述符
 * @param seconds 唤醒延迟秒数
 * @return 0 成功，-1 失败
 */
int rtc_set_alarm(int rtc_fd, int seconds)
{
    struct rtc_time rtc_tm;
    struct tm tm_now;
    time_t t;

    // 读取当前 RTC 时间
    if (ioctl(rtc_fd, RTC_RD_TIME, &rtc_tm) == -1)
    {
        perror("RTC_RD_TIME");
        return -1;
    }

    // 转换为 time_t
    tm_now.tm_sec = rtc_tm.tm_sec;
    tm_now.tm_min = rtc_tm.tm_min;
    tm_now.tm_hour = rtc_tm.tm_hour;
    tm_now.tm_mday = rtc_tm.tm_mday;
    tm_now.tm_mon = rtc_tm.tm_mon - 1;
    tm_now.tm_year = rtc_tm.tm_year + 100; // rtc_tm.tm_year 是从 1900 开始的偏移
    t = mktime(&tm_now);

    // 加上延迟秒数
    t += seconds;
    struct tm *tm_alarm = localtime(&t);

    // 填充闹钟结构
    struct rtc_wkalrm alarm;
    alarm.enabled = 1;
    alarm.time.tm_sec = tm_alarm->tm_sec;
    alarm.time.tm_min = tm_alarm->tm_min;
    alarm.time.tm_hour = tm_alarm->tm_hour;
    alarm.time.tm_mday = tm_alarm->tm_mday;
    alarm.time.tm_mon = tm_alarm->tm_mon;
    alarm.time.tm_year = tm_alarm->tm_year;
    alarm.time.tm_isdst = -1;

    // 设置闹钟
    if (ioctl(rtc_fd, RTC_WKALM_SET, &alarm) == -1)
    {
        perror("RTC_WKALM_SET");
        return -1;
    }

    // 启用 RTC 唤醒功能（若未默认启用）
    if (ioctl(rtc_fd, RTC_AIE_ON) == -1)
    {
        perror("RTC_AIE_ON");
        return -1;
    }

    printf("RTC alarm set for %d seconds later\n", seconds);
    return 0;
}

int rtc_set_time(struct rtc_time *rtc_tm)
{
    int fd = open("/dev/rtc0", O_RDWR);
    if (fd < 0)
    {
        perror("open /dev/rtc0");
        return -1;
    }
    if (ioctl(fd, RTC_SET_TIME, rtc_tm) == -1)
    {
        perror("RTC_SET_TIME ioctl");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int rtc_get_time(int fd, struct rtc_time *tm)
{
    if (ioctl(fd, RTC_RD_TIME, tm) == -1)
    {
        perror("RTC_RD_TIME");
        return -1;
    }
    return 0;
}

void rtc_clear_alarm(int rtc_fd)
{
    ioctl(rtc_fd, RTC_AIE_OFF);
    struct rtc_wkalrm alarm = {0};
    ioctl(rtc_fd, RTC_WKALM_SET, &alarm);
}