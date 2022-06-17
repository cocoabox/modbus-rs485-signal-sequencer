#pragma once
#include <avr/pgmspace.h>

const PROGMEM uint8_t p3_program[] = {
    // sec    ch0  ch1  ch2
    15, 1, 0, 0,
    15, 0, 0, 1,
    3, 0, 1, 0};

const uint8_t p3_num_channels = 3;
const uint8_t p3_num_steps = 3;