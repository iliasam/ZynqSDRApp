#ifndef SFIFO_H_
#define SFIFO_H_

#include <linux/kernel.h>

typedef struct {
    void *buf;

    uint32_t head; //in items
    uint32_t tail; //in items
    uint32_t item_size; //in bytes
    
    uint32_t amount; //in items
    uint32_t size; //in items
} sfifo_t;


// =============================== Declaration ================================

void sfifo_init(sfifo_t *fifo, void *buf, uint32_t entry_size_bytes, uint32_t fifo_size_items);
int sfifo_put(sfifo_t *fifo, void *item_p);
int sfifo_get(sfifo_t *fifo, void *item_p);

#endif /* SFIFO_H_ */

/** @} */
