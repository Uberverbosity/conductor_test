#pragma once
#include <lvgl.h>

// Create the page
void dial_page_create(lv_obj_t* parent);

// Relative change (encoder detents)
void dial_page_set_delta(int delta);

// Absolute set (DSP sync on READY)
void dial_page_set_absolute(int value);

// Optional getter
int dial_page_get_value();

typedef enum {
    DIAL_SLOT_VOL1 = 0,
    DIAL_SLOT_VOL2,
    DIAL_SLOT_VOL3,
    DIAL_SLOT_VOL4,
    DIAL_SLOT_COUNT
} DialSlot;

void dial_page_set_slot(DialSlot slot);
void dial_page_set_color(DialSlot slot, uint8_t r, uint8_t g, uint8_t b);
void dial_page_set_label(const char* text);
