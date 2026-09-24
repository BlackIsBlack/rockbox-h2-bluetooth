#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool bt_scan(void);
bool bt_heap_info(void);
size_t bt_read_pcm(uint8_t *buf, size_t len);