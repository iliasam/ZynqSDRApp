#include "sfifo.h"
#include <linux/string.h>


void sfifo_init(sfifo_t *fifo, void *buf, uint32_t item_size_bytes, uint32_t fifo_size_items)
{
    fifo->head = fifo->tail = fifo->amount = 0;
    fifo->buf = buf;
    fifo->size = fifo_size_items;
    fifo->item_size = item_size_bytes;
}

int sfifo_put(sfifo_t *fifo, void *item_p)
{
    if (fifo->amount >= fifo->size)
    {
        return -1;
    }
    else
    {
        memcpy((uint8_t*)fifo->buf + fifo->head * fifo->item_size, item_p, fifo->item_size);
        fifo->head++;

        if (fifo->head >= fifo->size)
        {
            fifo->head = 0;
        }

        fifo->amount++;
    }

    return 0;
}

int sfifo_get(sfifo_t *fifo, void *item_p)
{
    if (fifo->amount == 0)
    {
        return -1;
    }
    else
    {
        memcpy(item_p, (uint8_t*)fifo->buf + fifo->tail * fifo->item_size, fifo->item_size);
        fifo->tail++;

        if (fifo->tail >= fifo->size)
        {
            fifo->tail = 0;
        }

        fifo->amount--;
    }

    return 0;
}

/** @} */
