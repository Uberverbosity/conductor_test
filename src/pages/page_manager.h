#pragma once
#include <lvgl.h>

enum PageId {
    PAGE_VOL_0,
    PAGE_VOL_1,
    PAGE_VOL_2,
    PAGE_VOL_3,
    PAGE_TONE_LOW,
    PAGE_TONE_HIGH,
    PAGE_PRESET,
    PAGE_COUNT
};

void page_manager_init(lv_obj_t* root);
void page_manager_next();
void page_manager_prev();
void page_manager_refresh();
void page_manager_encoder_delta(int delta);
void page_manager_encoder_button();
void page_manager_encoder_long_press();
void page_manager_set_to_master_volume();