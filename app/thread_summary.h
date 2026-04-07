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
#include "llcc68.h"

void handle_signal(int sig);
void *receive_thread(void *arg);
void *sensor_send_thread(void *arg);
void *read_rtc_thread(void *arg);
void *write_file_thread(void *arg);
void *bd_send_thread(void *arg);
void *lora_transform_thread(void *arg);