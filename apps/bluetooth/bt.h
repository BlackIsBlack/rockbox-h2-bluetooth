#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool bt_scan(void);
/* 0 = off, 1 = on, 2 = connected, 3 = streaming (skin tag %Bt) */
int bt_get_status_level(void);
bool bt_heap_info(void);
size_t bt_read_pcm(uint8_t *buf, size_t len);