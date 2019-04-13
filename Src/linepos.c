#include "linepos.h"

#include <string.h>

static const float DIST_OPTO_CM = 22.472f / 31;
static const float MID_OPTO_POS = NUM_OPTOS / 2 + 0.5f;

static inline float opto_idx_to_pos_cm(float optoIdx) {
    return (optoIdx - MID_OPTO_POS) * DIST_OPTO_CM;
}

static inline uint8_t pos_cm_to_opto_idx(float pos_cm) {
    return (uint8_t)round(MID_OPTO_POS + pos_cm / DIST_OPTO_CM);
}

void linepos_calc(LinePosCalc *data, const uint8_t *measurements) {

    static const float comp_w_black = 0.75f;                // Weight of the black component when recalculating comparator level.
    static const float comp_w_white = 1.0f - comp_w_black;  // Weight of the white component when recalculating comparator level.

    uint32_t sumWhite = 0;  // The sum of the measurements under the comparator level.
    uint8_t cntrWhite = 0;  // Counts the values under the comparator level.

    uint32_t sumBlack = 0;  // The sum of the measurements above the comparator level.
    uint8_t cntrBlack = 0;  // Counts the values above the comparator level.

    uint32_t sumBlackLocal = 0;  // The sum of the measurements above the comparator level corresponding to the same line.
    uint32_t sumBlackLocalW = 0; // The weighted sum of the measurements above the comparator level corresponding to the same line.
    uint8_t cntrBlackLocal = 0;  // Counts the values above the comparator level corresponding to the same line.

    uint8_t prevIsBlack = 0;

    data->lines.numLines = 0;

    for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
        const uint8_t meas = measurements[i];
        const uint8_t isBlack = meas >= data->comparator;

        if (isBlack) {
            if (!prevIsBlack) { // rising edge - new line
                sumBlackLocal = 0;
                sumBlackLocalW = 0;
                cntrBlackLocal = 0;
            }

            sumBlack += meas;
            sumBlackLocal += meas;
            sumBlackLocalW += meas * i;

            ++cntrBlack;
            ++cntrBlackLocal;
        } else {
            if (prevIsBlack) {  // falling edge - line ended
                if (cntrBlackLocal > 1) {   // ignores single high values (noise filtering)
                    const float optoPos = (float)sumBlackLocalW / sumBlackLocal;
                    if (data->lines.numLines < MAX_LINES) {
                        data->lines.positions_cm[data->lines.numLines++] = opto_idx_to_pos_cm(optoPos);
                    }
                }
            }

            sumWhite += meas;
            ++cntrWhite;
        }

        prevIsBlack = isBlack;
    }

    // updates comparator if there were black and white values as well
    if (cntrWhite > 0 && cntrBlack > 0) {
        data->avgWhite = sumWhite / cntrWhite;
        data->avgBlack = sumBlack / cntrBlack;
        data->comparator = (uint8_t)(data->avgWhite * comp_w_white + data->avgBlack * comp_w_black);
    }
}

void linepos_set_leds(LinePosCalc *data, uint8_t *leds) {
    static uint8_t idx = 0;

    static int dir = 0;
    if (idx == 0) dir = 0;
    else if (idx == 31) dir = 1;
//    memset(leds, 0, NUM_OPTOS / 8);
//    for (uint8_t i = 0; i < data->lines.numLines; ++i) {
//        const uint8_t optoIdx = pos_cm_to_opto_idx(data->lines.positions_cm[i]);
//        leds[optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
//    }

    uint8_t ledIdx = 3 - idx / 8;

    for (uint8_t i = 0; i < 4; ++i) {
        if (i == ledIdx) {
            leds[i] = 1 << (idx % 8);
        } else {
            leds[i] = 0x00;
        }
    }

    idx = dir ? idx - 1 : idx + 1;
}
