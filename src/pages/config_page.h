#pragma once
#include <lvgl.h>

// UI lifecycle
void config_page_create(lv_obj_t* parent);
void config_page_on_enter(void);
void config_page_refresh(void);

// Encoder interactions
void config_page_delta(int delta);
void config_page_select();
bool config_page_back();
