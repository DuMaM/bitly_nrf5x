#include <app_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>

// based on
// https://nestedsoftware.com/2018/03/20/calculating-a-moving-average-on-streaming-data-5a7k.22879.html

static uint32_t timestamp;

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
    return (u24_val >= 0x800000u ? u24_val | 0xFF000000u : u24_val);
}

inline uint32_t conv_i32_to_u24(int32_t i32_val)
{
    return (uint32_t)(i32_val & 0xFFFFFFu);
}

inline uint8_t *conv_u24_to_raw(uint32_t u24_val, uint8_t *raw, uint16_t pos)
{
    raw[pos + 0] = (uint8_t)(0xFF & (u24_val >> 16));
    raw[pos + 1] = (uint8_t)(0xFF & (u24_val >> 8));
    raw[pos + 2] = (uint8_t)(0xFF & (u24_val >> 0));
    return raw + pos + 3;
}

void utils_reset_timestamp() {
    timestamp = k_cycle_get_32();
}

uint8_t* utils_write_timestamp(uint8_t* data) {
    static uint32_t end_time;

    end_time = k_cycle_get_32();
    return conv_u24_to_raw(k_cyc_to_us_ceil32(end_time), data, 0);
}

int8_t atob(const char *buffer)
{
#define lookup_size 4
    const static char *lookup_negative[lookup_size] = {"false", "0", "n", "no"};
    const static char *lookup_positive[lookup_size] = {"true", "1", "y", "yes"};

    for (int i = 0; i < lookup_size; i++)
    {
        if (!strcmp(lookup_negative[i], buffer))
            return 0;

        if (!strcmp(lookup_positive[i], buffer))
            return 1;
    }

    return -1;
}
