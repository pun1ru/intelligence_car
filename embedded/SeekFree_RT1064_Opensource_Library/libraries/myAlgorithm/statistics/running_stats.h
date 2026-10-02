#ifndef RUNNING_STATS_H
#define RUNNING_STATS_H

#include <stdint.h>

typedef struct
{
    uint32_t count;
    float mean[3];
    float m2[3];
} running_stats3_t;

void running_stats3_reset(running_stats3_t *stats);
void running_stats3_add(running_stats3_t *stats, const float sample[3]);
void running_stats3_variance(const running_stats3_t *stats, float variance[3]);

#endif
