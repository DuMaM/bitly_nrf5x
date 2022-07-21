#ifndef _MOVING_AVG_H_
#define _MOVING_AVG_H_

#include <zephyr/types.h>

typedef struct dynamic_mean {
    uint64_t count;
    int64_t mean;
} dynamic_mean_t;

void dm_update(dynamic_mean_t* dm, uint64_t new_value);

#endif // _MOVING_AVG_H_