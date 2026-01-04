#pragma once
#include <lvgl.h>
#include "protocol/helix_protocol.h"

// Create the page
void volume_page_create(lv_obj_t* parent);

// Relative change (encoder detents)
void volume_page_delta(int delta);

// Absolute set (DSP sync on READY)
void volume_page_set_absolute(int value);

// Optional getter
int volume_page_get_value();

void volume_page_set_slot(DialSlot slot);
void volume_page_set_color(DialSlot slot, uint8_t r, uint8_t g, uint8_t b);
void volume_page_set_label(const char* text);
void volume_page_on_enter();
void volume_page_refresh();

