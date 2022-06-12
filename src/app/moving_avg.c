#include <moving_avg.h>

// based on
// https://nestedsoftware.com/2018/03/20/calculating-a-moving-average-on-streaming-data-5a7k.22879.html

void dm_update(dynamic_mean_t* dm, uint64_t new_value) {
    dm->count++;
    uint64_t differential = (new_value - dm->mean) / dm->count;
    uint64_t new_mean = dm->mean + differential;
    dm->mean = new_mean;
}


