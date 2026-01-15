#include "config_page.h"
#include <Arduino.h>
#include <lvgl.h>
#include "settings.h"

// ================= INTERNAL UI OBJECTS =================

static lv_obj_t* list_container = nullptr;
static lv_obj_t* list_labels[3] = {nullptr, nullptr, nullptr};
static lv_timer_t* flash_timer = nullptr;

// ================= STYLING =================

static const lv_color_t BG_COLOR = lv_color_hex(0x000000);
static const lv_color_t SELECTED_COLOR = lv_color_hex(0xFFFFFF);  // Bright white
static const lv_color_t UNSELECTED_COLOR = lv_color_hex(0x777777); // Dimmer gray
static const lv_color_t FLASH_COLOR = lv_color_hex(0x1A1A1A);

// List items
static const char* CONFIG_ITEMS[] = {
    "Auto Return Time",
    "Dimming",
    "Theme"
};

static const char* THEME_ITEMS[] = {
    "Basic",
    "Mcintosh"
};

// ================= STATE =================

enum ConfigMenuLevel {
    CONFIG_MENU_ROOT,
    CONFIG_MENU_THEME
};

static ConfigMenuLevel menu_level = CONFIG_MENU_ROOT;
static int selected_index = 0;
static const char** active_items = CONFIG_ITEMS;
static int active_item_count = 3;

// ================= INTERNAL HELPERS =================

static void update_list_display()
{
    if (!list_container)
        return;

    const int LABEL_SPACING = 40;  // Vertical spacing between list items

    for (int i = 0; i < 3; i++) {
        if (!list_labels[i])
            continue;

        if (i < active_item_count) {
            lv_obj_clear_flag(list_labels[i], LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(list_labels[i], active_items[i]);

            int offset = (i - selected_index) * LABEL_SPACING;
            lv_obj_align(list_labels[i], LV_ALIGN_CENTER, 0, offset);

            lv_color_t color = (i == selected_index) ? SELECTED_COLOR : UNSELECTED_COLOR;
            lv_obj_set_style_text_color(list_labels[i], color, 0);
        } else {
            lv_obj_add_flag(list_labels[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void flash_cb(lv_timer_t*)
{
    if (list_container) {
        lv_obj_set_style_bg_color(list_container, BG_COLOR, 0);
    }
    flash_timer = nullptr;
}

static void flash_selection()
{
    if (!list_container)
        return;

    if (flash_timer) {
        lv_timer_del(flash_timer);
        flash_timer = nullptr;
    }

    lv_obj_set_style_bg_color(list_container, FLASH_COLOR, 0);
    flash_timer = lv_timer_create(flash_cb, 120, nullptr);
    lv_timer_set_repeat_count(flash_timer, 1);
}

static void set_menu_level(ConfigMenuLevel level)
{
    menu_level = level;

    if (menu_level == CONFIG_MENU_THEME) {
        active_items = THEME_ITEMS;
        active_item_count = 2;
    } else {
        active_items = CONFIG_ITEMS;
        active_item_count = 3;
    }

    if (menu_level == CONFIG_MENU_THEME) {
        UiTheme theme = settings_get_theme();
        selected_index = (theme == THEME_MCINTOSH) ? 1 : 0;
    } else {
        selected_index = 0;
    }
    update_list_display();
}

// ================= PUBLIC API =================

void config_page_create(lv_obj_t* parent)
{
    list_container = parent;

    lv_obj_set_style_bg_color(parent, BG_COLOR, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    for (int i = 0; i < 3; i++) {
        list_labels[i] = lv_label_create(parent);
        lv_obj_set_style_text_align(list_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(list_labels[i], &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(list_labels[i], UNSELECTED_COLOR, 0);
        lv_obj_center(list_labels[i]);
    }

    set_menu_level(CONFIG_MENU_ROOT);
}

void config_page_on_enter(void)
{
    set_menu_level(CONFIG_MENU_ROOT);
}

void config_page_refresh(void)
{
    update_list_display();
}

void config_page_delta(int delta)
{
    int next = selected_index + delta;

    if (next < 0) next = 0;
    if (next > (active_item_count - 1)) next = active_item_count - 1;

    if (next == selected_index)
        return;

    selected_index = next;
    update_list_display();
}

void config_page_select()
{
    if (menu_level == CONFIG_MENU_ROOT) {
        if (selected_index == 2) {
            set_menu_level(CONFIG_MENU_THEME);
            flash_selection();
            return;
        }
        flash_selection();
        return;
    }

    if (menu_level == CONFIG_MENU_THEME) {
        UiTheme next_theme = (selected_index == 1) ? THEME_MCINTOSH : THEME_BASIC;
        settings_set_theme(next_theme);
        flash_selection();
    }
}

bool config_page_back()
{
    if (menu_level == CONFIG_MENU_THEME) {
        set_menu_level(CONFIG_MENU_ROOT);
        return true;
    }

    return false;
}
