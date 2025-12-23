#pragma once
#include <lvgl.h>

enum PageId {
    PAGE_DIAL = 0,
    PAGE_TONE,
    PAGE_COUNT
};

void page_manager_init(lv_obj_t* parent);
void page_show(PageId id);
PageId page_current();
