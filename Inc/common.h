#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>
#include <stdbool.h>

int32_t round_to_int(float value);

int32_t clamp(int32_t value, int32_t min_val, int32_t max_val);

void array_average_min_max(const uint8_t *data, uint32_t size, uint8_t *avg_, uint8_t *min_, uint8_t *max_);

#define COMPARE_FUNCTION(_abbr_) int compare_ ## _abbr_(const void *a, const void *b);

COMPARE_FUNCTION(u8)
COMPARE_FUNCTION(u16)
COMPARE_FUNCTION(u32)
COMPARE_FUNCTION(i8)
COMPARE_FUNCTION(i16)
COMPARE_FUNCTION(i32)
COMPARE_FUNCTION(float)

#endif /* COMMON_H_ */
