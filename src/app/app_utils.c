#include <app_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// based on
// https://nestedsoftware.com/2018/03/20/calculating-a-moving-average-on-streaming-data-5a7k.22879.html

void dm_update(dynamic_mean_t *dm, uint64_t new_value)
{
    dm->count++;
    uint64_t differential = (new_value - dm->mean) / dm->count;
    uint64_t new_mean = dm->mean + differential;
    dm->mean = new_mean;
}


inline uint32_t conv_raw_to_u24(uint8_t *raw, uint16_t pos)
{
    return (((uint32_t)raw[pos + 0]) << 16) +
           (((uint32_t)raw[pos + 1]) << 8) +
           (((uint32_t)raw[pos + 2]) << 0);
}

inline int32_t conv_u24_to_i32(uint32_t u24_val)
{
    return ((int32_t)(u24_val << 8)) >> 8;
}

inline uint32_t conv_i32_to_u24(int32_t i32_val)
{
    return ((uint32_t)(i32_val << 8)) >> 8;
}

inline uint8_t *conv_u24_to_raw(uint32_t u24_val, uint8_t *raw, uint16_t pos)
{
    raw[pos + 0] = (uint8_t)(0xFF & (u24_val >> 16));
    raw[pos + 1] = (uint8_t)(0xFF & (u24_val >> 8));
    raw[pos + 2] = (uint8_t)(0xFF & (u24_val >> 0));
    return raw + pos + 3;
}
