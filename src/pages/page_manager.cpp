#include "page_manager.h"
#include "dial_page.h"
//#include "tone_page.h"

static lv_obj_t* page_root[PAGE_COUNT];
static PageId current_page = PAGE_DIAL;

void page_manager_init(lv_obj_t* parent)
{
    // Create page roots
    for (int i = 0; i < PAGE_COUNT; i++) {
        page_root[i] = lv_obj_create(parent);
        lv_obj_set_size(page_root[i], LV_PCT(100), LV_PCT(100));
        lv_obj_clear_flag(page_root[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(page_root[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Create page contents
    dial_page_create(page_root[PAGE_DIAL]);
    //tone_page_create(page_root[PAGE_TONE]);

    // Show initial page
    page_show(PAGE_DIAL);
}

void page_show(PageId id)
{
    if (id >= PAGE_COUNT || id == current_page)
        return;

    lv_obj_add_flag(page_root[current_page], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(page_root[id], LV_OBJ_FLAG_HIDDEN);

    current_page = id;
}

PageId page_current()
{
    return current_page;
}
