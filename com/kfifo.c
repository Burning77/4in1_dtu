#include <string.h>
#include "kfifo.h"

static unsigned int min(unsigned int a, unsigned int b)
{
    return a > b ? b : a;
}

/* static unsigned int max(unsigned int a, unsigned int b)
{
    return a > b ? a : b;
}
 */

inline unsigned int kfifo_len(struct kfifo *fifo)
{
    return fifo->in - fifo->out;
}

inline unsigned int kfifo_left(struct kfifo *fifo)
{

    return fifo->size - (fifo->in - fifo->out);
}

inline int kfifo_isempty(struct kfifo *fifo)
{
    return fifo->in == fifo->out;
}

inline int kfifo_isfull(struct kfifo *fifo)
{
    return fifo->size == (fifo->in - fifo->out);
}

void kfifo_init(struct kfifo *fifo, unsigned char *buffer, unsigned int size)
{
    fifo->buffer = buffer;
    fifo->size = size;
    fifo->in = 0;
    fifo->out = 0;
}

inline void kfifo_reset(struct kfifo *fifo)
{
    fifo->in = 0;
    fifo->out = 0;
}

inline unsigned int kfifo_put(struct kfifo *fifo, unsigned char *buffer, unsigned int len)
{
    unsigned int l;
    len = min(len, fifo->size - fifo->in + fifo->out);
    /* first put the data starting from fifo->in to buffer end */
    l = min(len, fifo->size - (fifo->in & (fifo->size - 1)));
    memcpy(fifo->buffer + (fifo->in & (fifo->size - 1)), buffer, l);
    /* then put the rest (if any) at the beginning of the buffer */
    memcpy(fifo->buffer, buffer + l, len - l);

    fifo->in += len;
    return len;
}

inline unsigned int kfifo_get(struct kfifo *fifo, unsigned char *buffer, unsigned int len)
{
    unsigned int l;
    len = min(len, fifo->in - fifo->out);

    /* first get the data from fifo->out until the end of the buffer */
    l = min(len, fifo->size - (fifo->out & (fifo->size - 1)));
    memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), l);
    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + l, fifo->buffer, len - l);
    /*
     * Ensure that we remove the bytes from the kfifo -before-
     * we update the fifo->out index.
     */
    fifo->out += len;
    return len;
}

inline unsigned int kfifo_get2(struct kfifo *fifo, unsigned char *buffer, unsigned int len)
{
    unsigned int l;
    len = min(len, fifo->in - fifo->out);

    /* first get the data from fifo->out until the end of the buffer */
    l = min(len, fifo->size - (fifo->out & (fifo->size - 1)));
    memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), l);
    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + l, fifo->buffer, len - l);
    /*
     * Ensure that we remove the bytes from the kfifo -before-
     * we update the fifo->out index.
     */
    return len;
}

inline void kfifo_skip(struct kfifo *fifo, unsigned int len)
{
    fifo->out += len;
}