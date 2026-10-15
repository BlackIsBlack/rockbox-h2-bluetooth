#pragma once
/* Rockbox libc has no struct timespec / clock_gettime; the HFP AG code only
 * uses them for coarse connection timestamps, so a tick-based one will do. */
#include <time.h>
#include "kernel.h"

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1

struct timespec {
    long tv_sec;
    long tv_nsec;
};

static inline int clock_gettime(int clk, struct timespec *ts)
{
    (void)clk;
    long t = current_tick;
    ts->tv_sec = t / HZ;
    ts->tv_nsec = (t % HZ) * (1000000000L / HZ);
    return 0;
}
#endif
