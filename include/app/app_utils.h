#ifndef _MOVING_AVG_H_
#define _MOVING_AVG_H_

#include <zephyr/types.h>

typedef struct dynamic_mean {
    uint64_t count;
    int64_t mean;
} dynamic_mean_t;

void dm_update(dynamic_mean_t* dm, uint64_t new_value);

uint32_t conv_raw_to_u24(uint8_t *raw, uint16_t pos);
int32_t conv_u24_to_i32(uint32_t u24_val);
uint32_t conv_i32_to_u24(int32_t i32_val);
uint8_t *conv_u24_to_raw(uint32_t u24_val, uint8_t *raw, uint16_t pos);

uint8_t* utils_write_timestamp(uint8_t* data);
void utils_reset_timestamp();
uint32_t utils_roundUp(uint32_t numToRound, uint32_t multiple);
int8_t atob(const char *buffer);

#endif // _MOVING_AVG_H_