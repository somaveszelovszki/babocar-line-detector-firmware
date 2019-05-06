#include "linepos.h"

#include <string.h>
#include <stdlib.h>

static const float DIST_OPTO_MM = 224.72f / 31;
static const float MID_OPTO_POS = NUM_OPTOS / 2 - 0.5f;

static inline int8_t opto_idx_to_pos_mm(float optoIdx) {
    return (int8_t)round_to_int((optoIdx - MID_OPTO_POS) * DIST_OPTO_MM);
}

static inline uint8_t pos_mm_to_opto_idx(float pos_mm) {
    return (uint8_t)clamp(round_to_int(MID_OPTO_POS + pos_mm / DIST_OPTO_MM), 0, NUM_OPTOS - 1);
}

void linepos_initialize(LinePosCalc *data) {
    data->avgBlack = data->avgWhite = 0;
    data->comparator_black = 100;
    data->comparator_grey = 50;
    data->lines.numLines = 0;
    for (uint8_t i = 0; i < MAX_NUM_LINES; ++i) {
        data->lines.values[i].pos_mm = 0;
    }
}

void linepos_calc(LinePosCalc *data, const uint8_t *measurements) {

    typedef enum {
        Color_WHITE = 0,
        Color_GREY,
        Color_BLACK
    } Color;

    static const float comp_w_black = 0.75f;    // Weight of the black component when recalculating BLACK comparator level.
    static const uint8_t grey_delta = 30;       // Difference between white average and GREY comparator level.

    uint32_t sumWhite = 0;      // The sum of the WHITE measurements (under the GREY comparator level).
    uint8_t cntrWhite = 0;      // Counts the WHITE values.

    uint32_t sumBlack = 0;      // The sum of the BLACK measurements (above the BLACK comparator level).
    uint8_t cntrBlack = 0;      // Counts the BLACK values.

    uint32_t sumDarkLocal = 0;  // The sum of the GREY or BLACK measurements corresponding to the same line.
    uint32_t sumDarkLocalW = 0; // The weighted sum of the GREY or BLACK measurements corresponding to the same line.
    uint8_t cntrBlackLocal = 0; // Counts the BLACK values corresponding to the same line.
    uint8_t cntrDarkLocal = 0;  // Counts the dark values corresponding to the same line.

    Color prevColor = Color_WHITE;

    data->lines.numLines = 0;

    for (uint8_t i = 0; i < NUM_OPTOS + 1; ++i) {
        const uint8_t meas = i < NUM_OPTOS ? measurements[i] : 0;
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
                ++cntrBlack;
                sumBlack += meas;
            }

            sumDarkLocal += meas;
            sumDarkLocalW += meas * (uint32_t)i;
            ++cntrDarkLocal;

        } else {
            if (prevColor >= Color_GREY) {  // falling edge - line ended
                if (cntrBlackLocal >= 2 || (cntrDarkLocal >= 2 && sumDarkLocal >= data->comparator_black)) {   // ignores single high values (noise filtering)
                    const float optoPos = (float)sumDarkLocalW / sumDarkLocal;
                    if (data->lines.numLines < MAX_NUM_LINES) {
                        data->lines.values[data->lines.numLines++].pos_mm = opto_idx_to_pos_mm(optoPos);
                    }
                }
            }

            sumWhite += meas;
            ++cntrWhite;
        }

        prevColor = color;
    }

    // updates comparator if there were dark and white values as well
    if (cntrWhite > 0 && cntrBlack > 0) {
        data->avgWhite = sumWhite / cntrWhite;
        data->avgBlack = sumBlack / cntrBlack;
        data->comparator_black = (uint8_t)(data->avgWhite * (1.0f - comp_w_black) + data->avgBlack * comp_w_black);
        data->comparator_grey = data->avgWhite + grey_delta;
    }
}

void linepos_calc2(LinePosCalc *data, const uint8_t *measurements) {

    typedef enum {
        Color_WHITE = 0,
        Color_GREY,
        Color_BLACK
    } Color;

    static const uint8_t black_delta = 70;
    static const uint8_t grey_delta = black_delta / 2;

    uint32_t sumWhite = 0;      // The sum of the WHITE measurements (under the GREY comparator level).
    uint8_t cntrWhite = 0;      // Counts the WHITE values.

    uint32_t sumBlack = 0;      // The sum of the BLACK measurements (above the BLACK comparator level).
    uint8_t cntrBlack = 0;      // Counts the BLACK values.

    uint32_t sumDarkLocal = 0;  // The sum of the GREY or BLACK measurements corresponding to the same line.
    uint32_t sumDarkLocalW = 0; // The weighted sum of the GREY or BLACK measurements corresponding to the same line.
    uint8_t cntrBlackLocal = 0; // Counts the BLACK values corresponding to the same line.
    uint8_t cntrDarkLocal = 0;  // Counts the dark values corresponding to the same line.

    Color prevColor = Color_WHITE;

    data->lines.numLines = 0;

    uint8_t avg_, min_, max_;
    array_average_min_max(measurements, NUM_OPTOS, &avg_, &min_, &max_);

    if (max_ - min_ < black_delta) { // no peaks
        return;
    }

    for (uint8_t i = 0; i < NUM_OPTOS + 1; ++i) {
        const uint8_t meas = i < NUM_OPTOS ? measurements[i] : 0;
        const Color color = meas >= avg_ + black_delta ? Color_BLACK : meas >= avg_ + grey_delta ? Color_GREY : Color_WHITE;

        if (color >= Color_GREY) {
            if (prevColor == Color_WHITE) { // rising edge - new line
                sumDarkLocal = 0;
                sumDarkLocalW = 0;
                cntrBlackLocal = 0;
                cntrDarkLocal = 0;
            }

            if (color == Color_BLACK) {
                ++cntrBlackLocal;
                ++cntrBlack;
                sumBlack += meas;
            }

            sumDarkLocal += meas;
            sumDarkLocalW += meas * (uint32_t)i;
            ++cntrDarkLocal;

        } else {
            if (prevColor >= Color_GREY) {  // falling edge - line ended
                if (cntrBlackLocal >= 2 || (cntrDarkLocal >= 2 && sumDarkLocal >= data->comparator_black)) {   // ignores single high values (noise filtering)
                    const float optoPos = (float)sumDarkLocalW / sumDarkLocal;
                    if (data->lines.numLines < MAX_NUM_LINES) {
                        data->lines.values[data->lines.numLines++].pos_mm = opto_idx_to_pos_mm(optoPos);
                    }
                }
            }

            sumWhite += meas;
            ++cntrWhite;
        }

        prevColor = color;
    }
}

void linepos_calc3(LinePosCalc *data, const uint8_t *measurements) {

    typedef enum {
        Color_WHITE = 0,
        Color_GREY,
        Color_BLACK
    } Color;

    static const uint8_t black_delta = 70;

    uint32_t sumWhite = 0;      // The sum of the WHITE measurements (under the GREY comparator level).
    uint8_t cntrWhite = 0;      // Counts the WHITE values.

    uint32_t sumBlack = 0;      // The sum of the BLACK measurements (above the BLACK comparator level).
    uint8_t cntrBlack = 0;      // Counts the BLACK values.

    uint32_t sumDarkLocal = 0;  // The sum of the GREY or BLACK measurements corresponding to the same line.
    uint32_t sumDarkLocalW = 0; // The weighted sum of the GREY or BLACK measurements corresponding to the same line.
    uint8_t cntrBlackLocal = 0; // Counts the BLACK values corresponding to the same line.
    uint8_t cntrDarkLocal = 0;  // Counts the dark values corresponding to the same line.

    Color prevColor = Color_WHITE;

    data->lines.numLines = 0;

    uint8_t avg_, min_, max_;
    array_average_min_max(measurements, NUM_OPTOS, &avg_, &min_, &max_);

    if (max_ - min_ < black_delta) { // no peaks
        return;
    }

    for (uint8_t i = 0; i < NUM_OPTOS + 1; ++i) {
        const uint8_t meas = i < NUM_OPTOS ? measurements[i] : 0;
        const Color color = meas >= avg_ + black_delta ? Color_BLACK : Color_WHITE;

        if (color >= Color_GREY) {
            if (prevColor == Color_WHITE) { // rising edge - new line
                sumDarkLocal = 0;
                sumDarkLocalW = 0;
                cntrBlackLocal = 0;
                cntrDarkLocal = 0;
            }

            if (color == Color_BLACK) {
                ++cntrBlackLocal;
                ++cntrBlack;
                sumBlack += meas;
            }

            sumDarkLocal += meas;
            sumDarkLocalW += meas * (uint32_t)i;
            ++cntrDarkLocal;

        } else {
            if (prevColor >= Color_GREY) {  // falling edge - line ended
                if (cntrBlackLocal >= 1) {
                    const float optoPos = (float)sumDarkLocalW / sumDarkLocal;
                    if (data->lines.numLines < MAX_NUM_LINES) {
                        data->lines.values[data->lines.numLines++].pos_mm = opto_idx_to_pos_mm(optoPos);
                    }
                }
            }

            sumWhite += meas;
            ++cntrWhite;
        }

        prevColor = color;
    }
}

void linepos_set_leds(const Lines *lines, uint8_t *leds) {
    memset(leds, 0, NUM_OPTOS / 8);
    for (uint8_t i = 0; i < lines->numLines; ++i) {
        const uint8_t optoIdx = pos_mm_to_opto_idx(lines->values[i].pos_mm);
        leds[(NUM_OPTOS / 8 - 1) - optoIdx / 8] |= (1 << (optoIdx % 8));  // sets correspondent bit
    }
}

void linepos_set_display(const Lines *lines, char *str) {
    uint8_t lineIdx = 0;
    uint8_t strIdx = 0;
//    str[strIdx++] = '|';
//    for (uint8_t i = 0; i < NUM_OPTOS; ++i) {
//        if (lineIdx < lines->numLines && pos_mm_to_opto_idx(lines->positions_mm[lineIdx]) == i) {
//            str[strIdx++] = 'M';
//            lineIdx++;
//        } else {
//            str[strIdx++] = ' ';
//        }
//    }
//    str[strIdx++] = '|';
//    str[strIdx++] = '\n';

    itoa(lines->numLines, str, 10);
    int32_t pos = strlen(str);
    if (lines->numLines) {
        strcpy(&str[pos], ": ");
        pos = strlen(str);
        for (uint8_t i = 0; i < lines->numLines; ++i) {
            itoa(lines->values[i].pos_mm, &str[pos], 10);
            pos = strlen(str);
            if (i < lines->numLines - 1) {
                strcpy(&str[pos], ", ");
                pos = strlen(str);
            }
        }

        strcpy(&str[pos], " [mm]");
        pos = strlen(str);
    }

    strcpy(&str[pos], "\n");
}
