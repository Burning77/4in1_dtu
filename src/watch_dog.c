#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>
#include "../inc/watch_dog.h"
int watchdog_fd = -1;
int init_watchdog(void)
{
    watchdog_fd = open("/dev/watchdog", O_WRONLY);
    if (watchdog_fd < 0)
    {
        perror("open /dev/watchdog");
        return -1;
    }

    int timeout = 15; // 设置超时时间为15秒
    ioctl(watchdog_fd, WDIOC_SETTIMEOUT, &timeout);
    printf("Watchdog enabled, timeout = %d seconds.\n", timeout);

    return 0;
}
void cleanup_watchdog(void)
{
    if (watchdog_fd != -1)
    {
        // 写入 'V' 禁用看门狗
        write(watchdog_fd, "V", 1);
        close(watchdog_fd);
        watchdog_fd = -1;
        printf("Watchdog disabled and closed.\n");
    }
}