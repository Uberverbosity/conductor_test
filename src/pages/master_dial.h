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
