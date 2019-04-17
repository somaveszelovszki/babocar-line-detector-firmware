#include "linepos.h"

#include <string.h>

static const float DIST_OPTO_MM = 224.72f / 31;
static const float MID_OPTO_POS = NUM_OPTOS / 2 + 0.5f;

static inline int8_t opto_idx_to_pos_mm(float optoIdx) {
    return (int8_t)((optoIdx - MID_OPTO_POS) * DIST_OPTO_MM);
}

static inline uint8_t pos_mm_to_opto_idx(float pos_mm) {
    return (uint8_t)round(MID_OPTO_POS + pos_mm / DIST_OPTO_MM);
}

void linepos_initialize(LinePosCalc *data) {
    data->avgDark = data->avgWhite = 0;
    data->comparator_black = 128;
    data->comparator_grey = 100;
    data->lines.numLines = 0;
    for (uint8_t i = 0; i < MAX_LINES; ++i) {
        data->lines.positions_mm[i] = 0;
    }
}

void linepos_calc(LinePosCalc *data, const uint8_t *measurements) {

    typedef enum {
        Color_WHITE = 0,
        Color_GREY,
        Color_BLACK
    } Color;

    static const float comp_w_black = 0.5f;     // Weight of the black component when recalculating BLACK comparator level.
    static const float comp_w_grey  = 0.4f;     // Weight of the grey component when recalculating GREY comparator level.

    uint32_t sumWhite = 0;      // The sum of the WHITE measurements (under the GREY comparator level).
    uint8_t cntrWhite = 0;      // Counts the WHITE values.

    uint32_t sumDark = 0;       // The sum of the BLACK or GREY measurements (above the GREY comparator level).
    uint8_t cntrDark = 0;       // Counts the BLACK or GREY values.

    uint32_t sumDarkLocal = 0;  // The sum of the GREY or BLACK measurements corresponding to the same line.
    uint32_t sumDarkLocalW = 0; // The weighted sum of the GREY or BLACK measurements corresponding to the same line.
    uint8_t cntrBlackLocal = 0; // Counts the BLACK values corresponding to the same line.
    uint8_t cntrDarkLocal = 0;  // Counts the dark values corresponding to the same line.

    Color prevColor = Color_WHITE;

    data->lines.numLines = 0;

    for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
        const uint8_t meas = measurements[i];
        const Color color = meas >= data->comparator_black ? Color_BLACK : meas >= data->comparator_grey ? Color_GREY : Color_WHITE;

        if (color >= Color_GREY) {
            if (prevColor == Color_WHITE) { // rising edge - new line
                sumDarkLocal = 0;
                sumDarkLocalW = 0;
                cntrBlackLocal = 0;
                cntrDarkLocal = 0;
            }

            if (color == Color_BLACK) {
                ++cntrBlackLocal;
            }

            sumDark += meas;
            sumDarkLocal += meas;
            sumDarkLocalW += meas * i;
            ++cntrDark;
            ++cntrDarkLocal;

        } else {
            if (prevColor >= Color_GREY) {  // falling edge - line ended
                if (cntrBlackLocal >= 2 || cntrDarkLocal >= 3) {   // ignores single high values (noise filtering)
                    const float optoPos = (float)sumDarkLocalW / sumDarkLocal;
                    if (data->lines.numLines < MAX_LINES) {
                        data->lines.positions_mm[data->lines.numLines++] = opto_idx_to_pos_mm(optoPos);
                    }
                }
            }

            sumWhite += meas;
            ++cntrWhite;
        }

        prevColor = color;
    }

    // updates comparator if there were dark and white values as well
    if (cntrWhite > 0 && cntrDark > 0) {
        data->avgWhite = sumWhite / cntrWhite;
        data->avgDark = sumDark / cntrDark;
        data->comparator_black = (uint8_t)(data->avgWhite * (1.0f - comp_w_black) + data->avgDark * comp_w_black);
        data->comparator_grey = (uint8_t)(data->avgWhite * (1.0f - comp_w_grey) + data->avgDark * comp_w_grey);
    }
}

void linepos_set_leds(LinePosCalc *data, uint8_t *leds) {
    memset(leds, 0, NUM_OPTOS / 8);
    for (uint8_t i = 0; i < data->lines.numLines; ++i) {
        const uint8_t optoIdx = pos_mm_to_opto_idx(data->lines.positions_mm[i]);
        leds[(NUM_OPTOS / 8 - 1) - optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
    }
}
