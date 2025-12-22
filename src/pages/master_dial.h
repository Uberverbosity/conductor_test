#pragma once
#include <lvgl.h>

// Create the page
void master_dial_create(lv_obj_t* parent);

// Relative change (encoder detents)
void master_dial_set_delta(int delta);

// Absolute set (DSP sync on READY)
void master_dial_set_absolute(int value);

// Optional getter
int master_dial_get_value();

typedef enum {
    DIAL_SLOT_VOL1 = 0,
    DIAL_SLOT_VOL2,
    DIAL_SLOT_VOL3,
    DIAL_SLOT_VOL4,
    DIAL_SLOT_COUNT
} DialSlot;

void master_dial_set_slot(DialSlot slot);
void master_dial_set_color(DialSlot slot, uint8_t r, uint8_t g, uint8_t b);