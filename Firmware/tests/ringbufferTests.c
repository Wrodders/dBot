#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "../common/ringbuffer.h"

void test_ring_buffer() {
    RingBuffer rb = ringbuffer_create(RING_BUFFER_SIZE);

    // Test 1: Check if the ring buffer is initially empty
    assert(ringbuffer_empty(&rb) == 1);

    // Test 2: Add data to the ring buffer and check if it is not empty
    ringbuffer_put(&rb, 42);
    assert(ringbuffer_empty(&rb) == 0);

    // Test 3: Check if the data added to the buffer can be retrieved
    uint8_t data = ringbuffer_get(&rb);
    assert(data == 42);
    assert(ringbuffer_empty(&rb));

    // Test 4: Fill the ring buffer and check if it is full
    for (uint8_t i = 0; i < RING_BUFFER_SIZE - 1; i++) {
        ringbuffer_put(&rb, i);
    }
    assert(ringbuffer_full(&rb) == 1);

    // Test 5: Peek into the ring buffer and check the first element
    data = ringbuffer_peek(&rb);
    assert(data == 0);
    assert(ringbuffer_empty(&rb) == 0);

    // Test 6: Remove all elements from the ring buffer
    for (uint8_t i = 0; i < RING_BUFFER_SIZE - 1; ++i) {
        data = ringbuffer_get(&rb);
        assert(data == i);
    }
    assert(ringbuffer_empty(&rb) == 1);
    assert(ringbuffer_full(&rb) == 0);
}

int main() {
    printf("Running tests on ringbuffer.h ...\n");
    test_ring_buffer();

    printf("All tests passed!\n");

    return 0;
}
