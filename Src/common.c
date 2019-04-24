#include "common.h"

int32_t round(float value) {
    return (int32_t)(value + 0.5f);
}

int32_t clamp(int32_t value, int32_t min_val, int32_t max_val) {
    if (value <= min_val) return min_val;
    if (value >= max_val) return max_val;
    return value;
}
