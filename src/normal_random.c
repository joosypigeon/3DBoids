#include "normal_random.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Returns a normally distributed value with given mean and standard deviation
float random_normal(float mean, float stddev) {
    // Use Box-Muller transform
    float u1 = ((float)random() + 1.0f) / ((float)RAND_MAX + 2.0f);
    float u2 = ((float)random() + 1.0f) / ((float)RAND_MAX + 2.0f);

    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return z0 * stddev + mean;
}
