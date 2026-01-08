#pragma once
#include <stdint.h>
#include <lvgl.h>

// UI lifecycle
void preset_page_create(lv_obj_t* parent);
void preset_page_on_enter(void);
void preset_page_refresh(void);

// Encoder / button interactions
void preset_page_delta(int delta);      // dial rotation
void preset_page_select(void);           // long-press

// Protocol callback (FB 2B 06 ACK)
void preset_page_on_ack(uint8_t idx);