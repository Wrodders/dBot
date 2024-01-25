#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#define RING_BUFFER_SIZE 64

typedef struct RingBuffer{
    char buf[RING_BUFFER_SIZE];
    int size;
    int head;
    int tail;
}RingBuffer;



static RingBuffer ringbuffer_create(int size){
    //@Breif: Create Ring Buffer
    RingBuffer rb = {0};
    rb.size = size;
    return rb;
}

static void ringbuffer_put( RingBuffer *rb, const int data){
    //@Breif: Add one byte to ring buffer
    rb->buf[rb->head++] = data;
    rb->head = rb->head == rb->size ? 0 : rb->head; // reset head if end
    return;
}

static int ringbuffer_get(RingBuffer *rb){
    //@Breif: Remove one byte from ring buffer
    const int data = rb->buf[rb->tail++];
    rb->tail = rb->tail == rb->size ? 0 : rb->tail; // reset tail if end
    return data;
}

static int ringbuffer_peek(const RingBuffer *rb){
    //@Breif: Get one byte from ring buffer withought removing it
    return rb->buf[rb->tail];
}
static bool ringbuffer_empty(const RingBuffer *rb){
    //@Breif: Check if ring buffer is empty
    return rb->tail == rb->head;
}
static bool ringbuffer_full(const RingBuffer *rb){
    //@Breif: Check if ring buffer is full
    int nextIdx = rb->head + 1;
    if(nextIdx == rb->size){
        nextIdx = 0;
    }
    return nextIdx == rb->tail; 
}

#endif // RING_BUFFER_H