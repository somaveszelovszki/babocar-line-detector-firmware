#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

#define NUM_OPTOS 32

int32_t round(float value);

int32_t clamp(int32_t value, int32_t min_val, int32_t max_val);

#endif /* COMMON_H_ */
