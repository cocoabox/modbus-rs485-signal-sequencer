#pragma once
#include <avr/pgmspace.h>


const PROGMEM uint8_t p2_program[] = {
    // sec  ch0  ch1  ch2
    6,      0,   0,   1,
    6,      0,   0,   3,
    3,      1,   0,   0,
    15,     1,   0,   0,
};

const uint8_t p2_num_channels = 3;
const uint8_t p2_num_steps = 4;