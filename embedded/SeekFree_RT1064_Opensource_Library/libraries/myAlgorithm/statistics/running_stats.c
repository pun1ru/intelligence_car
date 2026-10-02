#include "running_stats.h"

#include <stddef.h>
#include <string.h>

void running_stats3_reset(running_stats3_t *stats)
{
    if (stats != NULL)
    {
        memset(stats, 0, sizeof(*stats));
    }
}

void running_stats3_add(running_stats3_t *stats, const float sample[3])
{
    uint8_t axis;

    if ((stats == NULL) || (sample == NULL) || (stats->count == UINT32_MAX))
    {
        return;
    }

    stats->count++;
    for (axis = 0U; axis < 3U; axis++)
    {
        float delta = sample[axis] - stats->mean[axis];
        stats->mean[axis] += delta / (float)stats->count;
        stats->m2[axis] += delta * (sample[axis] - stats->mean[axis]);
    }
}

void running_stats3_variance(const running_stats3_t *stats, float variance[3])
{
    uint8_t axis;

    if ((stats == NULL) || (variance == NULL))
    {
        return;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        variance[axis] = (stats->count > 1U) ?
            stats->m2[axis] / (float)(stats->count - 1U) : 0.0f;
    }
}
