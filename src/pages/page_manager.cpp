#include "pages/page_manager.h"
#include "pages/volume_page.h"
#include "pages/tone_page.h"
#include "protocol/helix_protocol.h"
#include "pages/preset_page.h"

static lv_obj_t* root_obj;
static PageId current_page;

static const char* page_name(PageId p);

static bool page_available(PageId p)
{
    switch (p) {
        case PAGE_VOL_0: return helix_slot_valid(0);
        case PAGE_VOL_1: return helix_slot_valid(1);
        case PAGE_VOL_2: return helix_slot_valid(2);
        case PAGE_VOL_3: return helix_slot_valid(3);

        case PAGE_TONE_LOW:
        case PAGE_TONE_HIGH:
            return helix_tone_enabled();

        case PAGE_PRESET:
            return true;

        default:
            return false;
    }
}

static PageId next_page(PageId p, int dir)
{
    PageId start = p;
    PageId n = p;

    do {
        n = (PageId)((n + dir + PAGE_COUNT) % PAGE_COUNT);
        if (n == start) return p;
    } while (!page_available(n));

    return n;
}

static void create_page(PageId p)
{
    lv_obj_clean(root_obj);

    switch (p) {
        case PAGE_VOL_0:
        case PAGE_VOL_1:
        case PAGE_VOL_2:
        case PAGE_VOL_3:
            volume_page_create(root_obj);
            break;

        case PAGE_TONE_LOW:
            tone_page_create(root_obj, TONE_LOW);
            break;

        case PAGE_TONE_HIGH:
            tone_page_create(root_obj, TONE_HIGH);
            break;

        case PAGE_PRESET:
            preset_page_create(root_obj);
            break;
    }
}

static void enter_page(PageId p)
{
    switch (p) {
        case PAGE_VOL_0:
            helix_set_active_slot(0);
            volume_page_on_enter();
            break;
        case PAGE_VOL_1:
            helix_set_active_slot(1);
            volume_page_on_enter();
            break;
        case PAGE_VOL_2:
            helix_set_active_slot(2);
            volume_page_on_enter();
            break;
        case PAGE_VOL_3:
            helix_set_active_slot(3);
            volume_page_on_enter();
            break;

        case PAGE_TONE_LOW:
            tone_page_on_enter(TONE_LOW);
            break;

        case PAGE_TONE_HIGH:
            tone_page_on_enter(TONE_HIGH);
            break;
        case PAGE_PRESET:
            preset_page_on_enter();
            break;            
    }
}

void page_manager_init(lv_obj_t* root)
{
    root_obj = root;
    current_page = PAGE_VOL_0;
    create_page(current_page);
    enter_page(current_page);
    helix_ui_bind_complete();
}

void page_manager_next()
{
    current_page = next_page(current_page, +1);
    create_page(current_page);
    enter_page(current_page);
}

void page_manager_prev()
{
    current_page = next_page(current_page, -1);
    create_page(current_page);
    enter_page(current_page);
}

void page_manager_refresh()
{
    switch (current_page) {
        case PAGE_VOL_0:
        case PAGE_VOL_1:
        case PAGE_VOL_2:
        case PAGE_VOL_3:
            volume_page_refresh();
            break;

        case PAGE_TONE_LOW:
        case PAGE_TONE_HIGH:
            tone_page_refresh();
            break;
        case PAGE_PRESET:
            preset_page_refresh();
            break;    
        }
}

void page_manager_encoder_delta(int delta)
{
    switch (current_page) {

        case PAGE_VOL_0:
        case PAGE_VOL_1:
        case PAGE_VOL_2:
        case PAGE_VOL_3:
            volume_page_delta(delta);
            page_manager_refresh();
            break;

        case PAGE_TONE_LOW:
            tone_page_delta(TONE_LOW, delta);
            page_manager_refresh();
            break;

        case PAGE_TONE_HIGH:
            tone_page_delta(TONE_HIGH, delta);
            page_manager_refresh();
            break;

        case PAGE_PRESET:
            preset_page_delta(delta);
            break;
    }
}

void page_manager_encoder_button()
{
    page_manager_next();
}

void page_manager_encoder_long_press()
{
    switch (current_page) {
        case PAGE_PRESET:
            preset_page_select();
            break;

        default:
            break;
    }
}

