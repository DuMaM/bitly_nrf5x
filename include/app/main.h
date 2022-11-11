#ifndef _MAIN_H_
#define _MAIN_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#define MAIN_QUEUE_STACK_SIZE 4096
#define MAIN_QUEUE_PRIORITY 6

extern struct k_work_q main_work_q;

#endif // _MAIN_H_
