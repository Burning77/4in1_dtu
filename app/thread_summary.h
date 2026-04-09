#ifndef __THREAD_SUMMARY_H__
#define __THREAD_SUMMARY_H__
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

#define OFFSET_FILE_BD   "/home/cat/send_offset_bd.dat"
#define OFFSET_FILE_LORA "/home/cat/send_offset_lora.dat"
#define OFFSET_FILE_4G   "/home/cat/send_offset_4g.dat"
#define LORA_MAX_HEX_LEN 256

void handle_signal(int sig);
void *receive_thread(void *arg);
void *serial_send_thread(void *arg);
void *read_rtc_thread(void *arg);
void *write_file_thread(void *arg);
void *bd_send_thread(void *arg);
void *lora_transform_thread(void *arg);
#endif