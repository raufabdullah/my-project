#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used

// Add you code below

static uint8_t block_bitmap[BLOCK_COUNT];

void heap_init(void) {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        block_bitmap[i] = 0;
    }
}

void* heap_alloc(size_t size) {
    if (size <= 0) {
        return NULL;
    }

    int blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    for (int i = 0; i <= BLOCK_COUNT - blocks_needed; i++) {
        int free_count = 0;

        for (int j = i; j < i + blocks_needed; j++) {
            if (block_bitmap[j] == 0) {
                free_count++;
            } else {
                break;
            }
        }

        if (free_count == blocks_needed) {
            for (int j = i; j < i + blocks_needed; j++) {
                block_bitmap[j] = 1;
            }
            return HEAP_START_ADDR + (i * BLOCK_SIZE);
        }
    }
    return NULL;
}

void heap_free(void* ptr) {
    if (ptr == NULL || ptr < (void*)HEAP_START_ADDR || ptr >= (void*)(HEAP_START_ADDR + HEAP_SIZE)) {
        return;
    }

    int block_number = ((uint8_t*)ptr - HEAP_START_ADDR) / BLOCK_SIZE;

    if (((uint8_t*)ptr - HEAP_START_ADDR) % BLOCK_SIZE != 0) {
        return;
    }

    while (block_number < BLOCK_COUNT && block_bitmap[block_number] == 1) {
        block_bitmap[block_number] = 0;
        block_number++;
    }
}