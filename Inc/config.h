#ifndef CONFIG_H_
#define CONFIG_H_

#include "common.h"

#define NUM_OPTOS 32
#define MAX_LINE_JUMP_MM            10
#define MIN_LINE_SAMPLE_APPEAR      2
#define MIN_LINE_SAMPLE_DISAPPEAR   2

#define MAX_CMD_DELAY_MS            200u  // If no command is received for this amount of time, connection is lost

#endif /* CONFIG_H_ */
