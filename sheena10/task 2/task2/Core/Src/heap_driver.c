#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* === Heap Configuration === */
#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)   // 4 KB heap
#define BLOCK_SIZE       16           // each block is 16 bytes
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

/* === Compile-time Safety Check === */
#if (BLOCK_COUNT == 0)
#error "Invalid BLOCK_COUNT"
#endif

/* === Internal Metadata === */
// 0 = free, 1 = used
static uint8_t block_map[BLOCK_COUNT];     
// At allocation start index: number of blocks allocated
static uint16_t alloc_blocks[BLOCK_COUNT]; 

/* === Helper Functions === */

// Convert block index -> pointer inside heap
static inline void *block_index_to_ptr(uint16_t idx)
{
    return (void *)(HEAP_START_ADDR + ((uintptr_t)idx * BLOCK_SIZE));
}

// Check if pointer lies within heap
static inline int ptr_in_heap(const void *p)
{
    uintptr_t addr = (uintptr_t)p;
    uintptr_t start = (uintptr_t)HEAP_START_ADDR;
    return (addr >= start) && (addr < (start + HEAP_SIZE));
}

/* === API Functions === */

/**
 * @brief Initializes the heap by clearing metadata.
 */
void heap_init(void)
{
    memset(block_map, 0, sizeof(block_map));
    memset(alloc_blocks, 0, sizeof(alloc_blocks));
}

/**
 * @brief Allocates 'size' bytes from heap.
 * @param size Number of bytes to allocate.
 * @return Pointer to allocated memory or NULL if failed.
 */
void *heap_alloc(size_t size)
{
    if (size == 0)
        return NULL;

    uint16_t needed_blocks = (uint16_t)((size + BLOCK_SIZE - 1) / BLOCK_SIZE);
    if (needed_blocks == 0 || needed_blocks > BLOCK_COUNT)
        return NULL;

    uint16_t consecutive = 0;

    for (uint16_t i = 0; i < BLOCK_COUNT; ++i)
    {
        if (block_map[i] == 0)
        {
            consecutive++;
            if (consecutive == needed_blocks)
            {
                uint16_t start = i + 1 - needed_blocks;

                // Mark as used
                for (uint16_t j = start; j < start + needed_blocks; ++j)
                    block_map[j] = 1;

                alloc_blocks[start] = needed_blocks;
                return block_index_to_ptr(start);
            }
        }
        else
        {
            consecutive = 0;
        }
    }

    // No sufficient space
    return NULL;
}

/**
 * @brief Frees a previously allocated block.
 * @param ptr Pointer previously returned by heap_alloc().
 */


void heap_free(void *ptr)
{
    if (ptr == NULL)
        return;

    if (!ptr_in_heap(ptr))
        return;

    uintptr_t base = (uintptr_t)HEAP_START_ADDR;
    uintptr_t p = (uintptr_t)ptr;

    // Ensure pointer is block aligned
    if ((p - base) % BLOCK_SIZE != 0)
        return;

    uint16_t idx = (uint16_t)((p - base) / BLOCK_SIZE);
    if (idx >= BLOCK_COUNT)
        return;

    uint16_t nblocks = alloc_blocks[idx];
    if (nblocks == 0)
        return; // invalid or double free

    // Free all associated blocks
    for (uint16_t i = idx; i < idx + nblocks && i < BLOCK_COUNT; ++i)
        block_map[i] = 0;

    alloc_blocks[idx] = 0;
}
