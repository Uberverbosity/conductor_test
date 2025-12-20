#pragma once
#include <lvgl.h>

// Public API for this page:

void master_dial_create(lv_obj_t* parent);
void master_dial_set_value(int delta);

// Optional getter, in case you want the dial value externally
int master_dial_get_value();
