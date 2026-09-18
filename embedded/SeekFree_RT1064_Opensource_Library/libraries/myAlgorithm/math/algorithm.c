#include "algorithm.h"

#include <math.h>

static uint16_t least_square_sample_count(const leastSquareLinear *data)
{
    uint16_t count;

    if (data->num > 0U)
    {
        count = data->num;
    }
    else if (data->valid_num > 0.0f)
    {
        count = (uint16_t)data->valid_num;
    }
    else
    {
        count = 0U;
    }

    return (count > 30U) ? 30U : count;
}

void SmoothFilterInitialize(SmoothFilter *filter, float alpha)
{
    if (filter == 0)
    {
        return;
    }

    filter->alpha = (alpha < 0.0f) ? 0.0f : ((alpha > 1.0f) ? 1.0f : alpha);
    filter->last = 0.0f;
    filter->current = 0.0f;
}

float SmoothFilterUpdate(SmoothFilter *filter, float input)
{
    if (filter == 0)
    {
        return input;
    }

    filter->last = filter->current;
    filter->current = filter->alpha * filter->last + (1.0f - filter->alpha) * input;
    return filter->current;
}

void AverageFilterInitialize(AverageFilter *filter)
{
    if (filter == 0)
    {
        return;
    }

    filter->last[0] = 0.0f;
    filter->last[1] = 0.0f;
    filter->last[2] = 0.0f;
}

float AverageFilterUpdate(AverageFilter *filter, float input)
{
    int index;

    if (filter == 0)
    {
        return input;
    }

    input = 0.1f * input + 0.4f * filter->last[2] +
            0.3f * filter->last[1] + 0.2f * filter->last[0];

    for (index = 2; index > 0; --index)
    {
        filter->last[index] = filter->last[index - 1];
    }
    filter->last[0] = input;
    return input;
}

float AngleLimit(float angle, float limit_min, float limit_max)
{
    const float stride = limit_max - limit_min;

    if (stride <= 0.0f)
    {
        return angle;
    }

    while (angle < limit_min)
    {
        angle += stride;
    }
    while (angle > limit_max)
    {
        angle -= stride;
    }

    return angle;
}

float AbsLimiter(float value, float max_value)
{
    const float limit = fabsf(max_value);

    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

float DoubleEdgeLimiter(float value, float min_value, float max_value)
{
    if (min_value > max_value)
    {
        const float swap = min_value;
        min_value = max_value;
        max_value = swap;
    }

    return (value > max_value) ? max_value :
           ((value < min_value) ? min_value : value);
}

float InvSqrt(float value)
{
    union
    {
        float value;
        uint32_t bits;
    } number;
    float half_value;

    if (value <= 0.0f)
    {
        return 0.0f;
    }

    half_value = 0.5f * value;
    number.value = value;
    number.bits = 0x5f3759dfU - (number.bits >> 1U);
    number.value = number.value * (1.5f - (half_value * number.value * number.value));
    return number.value;
}

int Sign(float value)
{
    if (value > 1.0e-9f)
    {
        return 1;
    }
    if (value < -1.0e-9f)
    {
        return -1;
    }
    return 0;
}

void leastSquareLinearFit(leastSquareLinear *data)
{
    uint16_t index;
    const uint16_t count = (data == 0) ? 0U : least_square_sample_count(data);
    float sum_x2 = 0.0f;
    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_xy = 0.0f;
    float denominator;

    if (count < 2U)
    {
        if (data != 0)
        {
            data->a = 0.0f;
            data->b = (count == 1U) ? data->y[0] : 0.0f;
        }
        return;
    }

    for (index = 0U; index < count; ++index)
    {
        sum_x2 += data->x[index] * data->x[index];
        sum_y += data->y[index];
        sum_x += data->x[index];
        sum_xy += data->x[index] * data->y[index];
    }

    denominator = (float)count * sum_x2 - sum_x * sum_x;
    if (fabsf(denominator) <= 1.0e-12f)
    {
        data->a = 0.0f;
        data->b = sum_y / (float)count;
        return;
    }

    data->a = ((float)count * sum_xy - sum_x * sum_y) / denominator;
    data->b = (sum_x2 * sum_y - sum_x * sum_xy) / denominator;
}

void improvedleastSquareLinearFit(leastSquareLinear *data, float lambda)
{
    uint16_t index;
    const uint16_t count = (data == 0) ? 0U : least_square_sample_count(data);
    float sum_w = 0.0f;
    float sum_wx = 0.0f;
    float sum_wy = 0.0f;
    float sum_wx2 = 0.0f;
    float sum_wxy = 0.0f;
    float denominator;
    float threshold;

    if (data == 0 || count == 0U)
    {
        return;
    }
    if (lambda <= 0.0f || lambda > 1.0f)
    {
        lambda = 1.0f;
    }

    for (index = 0U; index < count; ++index)
    {
        const float weight = powf(lambda, (float)index);
        const float x = data->x[index];
        const float y = data->y[index];

        sum_w += weight;
        sum_wx += weight * x;
        sum_wy += weight * y;
        sum_wx2 += weight * x * x;
        sum_wxy += weight * x * y;
    }

    denominator = sum_w * sum_wx2 - sum_wx * sum_wx;
    threshold = 1.0e-6f * sum_w * sum_w;
    if (fabsf(denominator) < threshold)
    {
        data->a = 0.0f;
        data->b = (sum_w > 0.0f) ? (sum_wy / sum_w) : 0.0f;
    }
    else
    {
        data->a = (sum_w * sum_wxy - sum_wx * sum_wy) / denominator;
        data->b = (sum_wx2 * sum_wy - sum_wx * sum_wxy) / denominator;
    }
}
