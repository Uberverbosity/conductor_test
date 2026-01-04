#pragma once
#include <Arduino.h>

void helix_begin(HardwareSerial& dsp);
void helix_loop();
bool helix_ready();

// ------ Volume slot identifiers ------
typedef enum {
    DIAL_SLOT_VOL1 = 0,
    DIAL_SLOT_VOL2,
    DIAL_SLOT_VOL3,
    DIAL_SLOT_VOL4,
    DIAL_SLOT_COUNT
} DialSlot;

enum ToneBand {
    TONE_LOW,
    TONE_HIGH
};

uint8_t helix_get_master_index();

void helix_volume_delta(int8_t clicks);
void helix_force_resync();
void helix_ui_bind_complete();

// ---- Volume slot state (read-only for UI) ----
DialSlot    helix_get_active_slot();
const char* helix_get_slot_label(DialSlot slot);
int         helix_get_slot_ui_value(DialSlot slot);
void helix_set_active_slot(uint8_t slot);
void helix_cycle_slot();
bool helix_slot_valid(uint8_t slot);

// ---- Tone capability / state (read-only for UI) ----
bool   helix_tone_enabled();
bool   helix_tone_low_valid();
bool   helix_tone_high_valid();
int8_t helix_tone_low_db();
int8_t helix_tone_high_db();
void helix_tone_delta(ToneBand band, int delta);