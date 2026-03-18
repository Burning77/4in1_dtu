/*
 * @Author: your name
 * @Date: 2021-06-03 20:14:14
 * @LastEditTime: 2021-06-08 10:57:24
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: \hddcs_payload.sdk\DCS\src\toolkits\kfifo.h
 */
#ifndef TOOLKITS_KFIFO_H_
#define TOOLKITS_KFIFO_H_

struct kfifo
{
    unsigned char *buffer;     /* the buffer holding the data */
    unsigned int size;         /* the size of the allocated buffer */
    volatile unsigned int in;  /* data is added at offset (in % size) */
    volatile unsigned int out; /* data is extracted from off. (out % size) */
};

void kfifo_init(struct kfifo *fifo, unsigned char *buffer, unsigned int size);

unsigned int kfifo_put(struct kfifo *fifo, unsigned char *buffer, unsigned int len);

unsigned int kfifo_get(struct kfifo *fifo, unsigned char *buffer, unsigned int len);

unsigned int kfifo_get2(struct kfifo *fifo, unsigned char *buffer, unsigned int len);

unsigned int kfifo_len(struct kfifo *fifo);

void kfifo_reset(struct kfifo *fifo);

unsigned int kfifo_left(struct kfifo *fifo);

int kfifo_isempty(struct kfifo *fifo);

void kfifo_skip(struct kfifo *fifo, unsigned int len);

#endif // KFIFO_H
