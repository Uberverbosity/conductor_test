#include "pages/page_manager.h"
#include "pages/volume_page.h"
#include "pages/tone_page.h"
#include "protocol/helix_protocol.h"
#include "pages/preset_page.h"
#include "pages/signal_input_page.h"
#include "pages/config_page.h"

static lv_obj_t* root_obj;
static PageId current_page;
static PageId last_page_before_config = PAGE_VOL_0;

static const char* page_name(PageId p);

static bool page_available(PageId p)
{
    if (p >= PAGE_VOL_0 && p <= PAGE_VOL_3) {
        return helix_slot_valid(p - PAGE_VOL_0);
    }

    switch (p) {
        case PAGE_TONE_LOW:
        case PAGE_TONE_HIGH:
            return helix_tone_enabled();

        case PAGE_PRESET:
            return true;

        case PAGE_SIGNAL_INPUT:
            return helix_signal_input_enabled();

        case PAGE_CONFIG:
            return false;

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

        case PAGE_SIGNAL_INPUT:
            signal_input_page_create(root_obj);
            break;

        case PAGE_CONFIG:
            config_page_create(root_obj);
            break;
    }
}

static void enter_page(PageId p)
{
    if (p >= PAGE_VOL_0 && p <= PAGE_VOL_3) {
        helix_set_active_slot(p - PAGE_VOL_0);
        volume_page_on_enter();
        return;
    }

    switch (p) {
        case PAGE_TONE_LOW:
            tone_page_on_enter(TONE_LOW);
            break;

        case PAGE_TONE_HIGH:
            tone_page_on_enter(TONE_HIGH);
            break;

        case PAGE_PRESET:
            preset_page_on_enter();
            break;

        case PAGE_SIGNAL_INPUT:
            signal_input_page_on_enter();
            break;

        case PAGE_CONFIG:
            config_page_on_enter();
            break;
    }
}

void page_manager_init(lv_obj_t* root)
{
    root_obj = root;
    current_page = PAGE_VOL_0;
    last_page_before_config = current_page;
    create_page(current_page);
    enter_page(current_page);
    // Note: helix_ui_bind_complete() is called separately to avoid double-init recursion
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
    if (current_page >= PAGE_VOL_0 && current_page <= PAGE_VOL_3) {
        volume_page_refresh();
    } else {
        switch (current_page) {
            case PAGE_TONE_LOW:
            case PAGE_TONE_HIGH:
                tone_page_refresh();
                break;
            case PAGE_PRESET:
                preset_page_refresh();
                break;
            case PAGE_SIGNAL_INPUT:
                signal_input_page_refresh();
                break;
            case PAGE_CONFIG:
                config_page_refresh();
                break;
        }
    }
}

void page_manager_encoder_delta(int delta)
{
    if (current_page >= PAGE_VOL_0 && current_page <= PAGE_VOL_3) {
        volume_page_delta(delta);
        page_manager_refresh();
    } else {
        switch (current_page) {
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

            case PAGE_SIGNAL_INPUT:
                signal_input_page_delta(delta);
                break;
            case PAGE_CONFIG:
                config_page_delta(delta);
                break;
        }
    }
    helix_note_user_interaction();
}

void page_manager_encoder_button()
{
    if (current_page == PAGE_CONFIG) {
        config_page_select();
    } else {
        page_manager_next();
    }
    helix_note_user_interaction();
}

void page_manager_encoder_double_click()
{
    if (current_page == PAGE_PRESET) {
        preset_page_select();
    }
    helix_note_user_interaction();
}

void page_manager_encoder_long_press()
{
    if (current_page == PAGE_CONFIG) {
        if (config_page_back()) {
            helix_note_user_interaction();
            return;
        }
        current_page = last_page_before_config;
    } else {
        last_page_before_config = current_page;
        current_page = PAGE_CONFIG;
    }
    create_page(current_page);
    enter_page(current_page);
    helix_note_user_interaction();
}

void page_manager_set_to_master_volume()
{
    if (current_page != PAGE_VOL_0) {
        current_page = PAGE_VOL_0;
        create_page(current_page);
        enter_page(current_page);
    }
}
