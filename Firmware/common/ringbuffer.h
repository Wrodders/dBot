#ifndef RING_BUFFER_H
#define RING_BUFFER_H

typedef struct Ring_Buffer{
    uint8_t *buffer;
    uint8_t size;
    uint8_t head;
    uint8_t tail;
}Ring_Buffer;

static void ringbuffer_put(Ring_Buffer *rb, const uint8_t data){
    //@Breif: Add one byte to ring buffer
    rb->buffer[rb->head++] = data;
    rb->head = rb->head == rb->size ? 0 : rb->head; // reset head if end
    return;
}

static uint8_t ringbuffer_get(Ring_Buffer *rb){
    //@Breif: Remove one byte from ring buffer
    const uint8_t data = rb->buffer[rb->tail++];
    rb->tail = rb->tail == rb->size ? 0 : rb->tail; // reset head if end
    return data;
}

static uint8_t ringbuffer_peek(const Ring_Buffer *rb){
    //@Breif: Get one byte from ring buffer withought removing it
    return rb->buffer[rb->tail];
}
static bool ringbuffer_empty(const Ring_Buffer *rb){
    //@Breif: Check if ring buffer is empty
    return rb->tail == rb->head;
}
static bool ringbuffer_full(const Ring_Buffer *rb){
    //@Breif: Check if ring buffer is full
    uint8_t nextIdx = rb->head + 1;
    if(nextIdx == rb->size){
        nextIdx = 0;
    }
    return nextIdx == rb->tail; 
}

#endif // RING_BUFFER_H