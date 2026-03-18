#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/rtc.h>
#include "../inc/rtc.h"

int rtc_set_time(struct rtc_time *rtc_tm) {
    int fd = open("/dev/rtc0", O_RDWR);
    if (fd < 0) {
        perror("open /dev/rtc0");
        return -1;
    }
    if (ioctl(fd, RTC_SET_TIME, rtc_tm) == -1) {
        perror("RTC_SET_TIME ioctl");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

int rtc_get_time(int fd, struct rtc_time *tm) {
    if (ioctl(fd, RTC_RD_TIME, tm) == -1) {
        perror("RTC_RD_TIME");
        return -1;
    }
    return 0;
}