#ifndef CONFIG_H_
#define CONFIG_H_

#include "common.h"

#define NUM_OPTOS                   32
#define MAX_LINE_JUMP_MM            25
#define MIN_LINE_DIST_MM            30
#define MIN_LINE_SAMPLE_APPEAR      7
#define MIN_LINE_SAMPLE_DISAPPEAR   7

#define OPTO_ARRAY_LENGTH_MM        224.72f
#define MIN_OPTO_POS_MM             (-OPTO_ARRAY_LENGTH_MM / 2)
#define MAX_OPTO_POS_MM             (OPTO_ARRAY_LENGTH_MM / 2)
#define DIST_OPTO_MM                (OPTO_ARRAY_LENGTH_MM / (NUM_OPTOS - 1))

#endif /* CONFIG_H_ */
