#pragma once
#include <lvgl.h>

// UI lifecycle
void signal_input_page_create(lv_obj_t* parent);
void signal_input_page_on_enter(void);
void signal_input_page_refresh(void);

// Encoder interactions
void signal_input_page_delta(int delta);
