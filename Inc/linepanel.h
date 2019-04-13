#ifndef LINEPANEL_H_
#define LINEPANEL_H_

#include "common.h"

#include "stm32f0xx_hal.h"

HAL_StatusTypeDef linepanel_initialize();

HAL_StatusTypeDef linepanel_read_optos(uint8_t *result);

HAL_StatusTypeDef linepanel_write_leds(const uint8_t *leds);

#endif /* LINEPANEL_H_ */
