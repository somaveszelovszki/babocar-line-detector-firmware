#include "common.h"

int32_t round_to_int(float value) {
    return (int32_t)(value + 0.5f);
}

int32_t clamp(int32_t value, int32_t min_val, int32_t max_val) {
    if (value <= min_val) return min_val;
    if (value >= max_val) return max_val;
    return value;
}

void array_average_min_max(const uint8_t *data, uint32_t size, uint8_t *avg_, uint8_t *min_, uint8_t *max_) {
    uint32_t sum = *min_ = *max_ = data[0];

    for (uint32_t i = 1; i < size; ++i) {
        const uint8_t elem = data[i];
        sum += elem;
        if (elem < *min_) *min_ = elem;
        if (elem > *max_) *max_ = elem;
    }

    *avg_ = sum / size;
}
