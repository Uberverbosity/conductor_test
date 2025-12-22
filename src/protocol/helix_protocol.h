#pragma once
#include <Arduino.h>

void helix_begin(HardwareSerial& dsp);
void helix_loop();
bool helix_ready();

uint8_t helix_get_master_index();

void helix_volume_delta(int8_t clicks);
void helix_force_resync();

void helix_set_active_slot(uint8_t slot);
uint8_t helix_get_active_slot();
void helix_cycle_slot();