#pragma once
#include <Arduino.h>

void helix_begin(HardwareSerial& dsp);
void helix_loop();

bool helix_ready();

// Encoder → DSP intent
void helix_volume_delta(int8_t clicks);
