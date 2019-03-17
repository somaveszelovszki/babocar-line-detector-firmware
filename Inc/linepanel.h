#ifndef LINEPANEL_H_
#define LINEPANEL_H_

#include "common.h"

void linepanel_initialize();

void linepanel_read_optos(uint8_t *result);

void linepanel_write_leds(const uint8_t *leds);

#endif /* LINEPANEL_H_ */
