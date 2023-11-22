#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#define RING_BUFFER_SIZE 16

typedef struct RingBuffer{
    uint8_t buf[RING_BUFFER_SIZE];
    uint8_t size;
    uint8_t head;
    uint8_t tail;
}RingBuffer;

static void ringbuffer_put( RingBuffer *rb, const uint8_t data){
    //@Breif: Add one byte to ring buffer
    rb->buf[rb->head++] = data;
    rb->head = rb->head == rb->size ? 0 : rb->head; // reset head if end
    return;
}

static uint8_t ringbuffer_get(RingBuffer *rb){
    //@Breif: Remove one byte from ring buffer
    const uint8_t data = rb->buf[rb->tail++];
    rb->tail = rb->tail == rb->size ? 0 : rb->tail; // reset head if end
    return data;
}

static uint8_t ringbuffer_peek(const RingBuffer *rb){
    //@Breif: Get one byte from ring buffer withought removing it
    return rb->buf[rb->tail];
}
static bool ringbuffer_empty(const RingBuffer *rb){
    //@Breif: Check if ring buffer is empty
    return rb->tail == rb->head;
}
static bool ringbuffer_full(const RingBuffer *rb){
    //@Breif: Check if ring buffer is full
    uint8_t nextIdx = rb->head + 1;
    if(nextIdx == rb->size){
        nextIdx = 0;
    }
    return nextIdx == rb->tail; 
}

#endif // RING_BUFFER_H