#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>
#include <math.h>

#define square(x) ((x) * (x))
#define cube(x) ((x) * (x) * (x))
#define fsgn(x) ((fabsf(x) < 1.0e-6f) ? 0 : (((x) > 0.0f) - ((x) < 0.0f)))

typedef struct
{
    float alpha;
    float last;
    float current;
} SmoothFilter;

typedef struct
{
    float last[3];
} AverageFilter;

typedef struct
{
    float x[30];
    float y[30];
    uint16_t num;
    float a;
    float b;
    float valid_num;
    uint8_t count;
} leastSquareLinear;

void SmoothFilterInitialize(SmoothFilter *filter, float alpha);
float SmoothFilterUpdate(SmoothFilter *filter, float input);

void AverageFilterInitialize(AverageFilter *filter);
float AverageFilterUpdate(AverageFilter *filter, float input);

float AngleLimit(float angle, float limit_min, float limit_max);
float AbsLimiter(float value, float max_value);
float DoubleEdgeLimiter(float value, float min_value, float max_value);
float InvSqrt(float value);
int Sign(float value);

void leastSquareLinearFit(leastSquareLinear *data);
void improvedleastSquareLinearFit(leastSquareLinear *data, float lambda);

#endif
