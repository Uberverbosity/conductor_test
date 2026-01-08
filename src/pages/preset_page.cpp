#include "preset_page.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>
#include "protocol/helix_protocol.h"
//#include "fonts/lv_font_montserrat_72.h"

// ================= INTERNAL UI OBJECTS =================

static lv_obj_t* dial_label_center = nullptr;
static lv_obj_t* dial_label_bottom = nullptr;

// ================= STYLING =================

static const lv_color_t COLOR_ACTIVE  = lv_color_hex(0xFFFFFF);
static const lv_color_t COLOR_FLASH   = lv_color_hex(0x666666);
static const lv_color_t BG_COLOR      = lv_color_hex(0x000000);

// ================= STATE =================

static uint8_t current_preset = 0;   // 0-based, confirmed active
static uint8_t pending_preset = 0;   // dial-selected, not yet applied
static bool    has_pending    = false;

// ================= INTERNAL HELPERS =================

static void update_center_label(uint8_t idx, lv_color_t color)
{
    if (!dial_label_center)
        return;

    char buf[4];
    snprintf(buf, sizeof(buf), "%u", idx + 1);  // display 1–8

    lv_label_set_text(dial_label_center, buf);
    lv_obj_set_style_text_color(dial_label_center, color, 0);
    lv_obj_center(dial_label_center);

}

static void flash_selected_preset(void)
{
    // Dark flash → white, non-blocking
    update_center_label(current_preset, COLOR_FLASH);
    lv_timer_t* t = lv_timer_create(
        [](lv_timer_t*) {
            update_center_label(current_preset, COLOR_ACTIVE);
        },
        200,
        nullptr
    );
    lv_timer_set_repeat_count(t, 1);
}

// ================= PUBLIC API =================

void preset_page_create(lv_obj_t* parent)
{
    // Background
    lv_obj_set_style_bg_color(parent, BG_COLOR, 0);

    // ----- CENTER LABEL -----
    dial_label_center = lv_label_create(parent);
 
    lv_obj_set_size(dial_label_center, 240, 100);
    lv_label_set_long_mode(dial_label_center, LV_LABEL_LONG_CLIP);

    lv_obj_center(dial_label_center);
    lv_obj_set_style_text_font(
        dial_label_center,
        &lv_font_montserrat_48,
        0
    );
    lv_obj_set_style_text_align(dial_label_center, LV_TEXT_ALIGN_CENTER, 0);
    //lv_obj_set_style_transform_zoom(dial_label_center, 333, 0); // ~1.30×
    lv_obj_set_style_text_color(
        dial_label_center,
        COLOR_ACTIVE,
        0
    );

    // ----- BOTTOM LABEL -----
    dial_label_bottom = lv_label_create(parent);
    lv_obj_set_style_text_font(
        dial_label_bottom,
        &lv_font_montserrat_20,
        0
    );
    lv_obj_set_style_text_color(
        dial_label_bottom,
        COLOR_ACTIVE,
        0
    );
    lv_obj_align(dial_label_bottom, LV_ALIGN_BOTTOM_MID, 0, -35);
    lv_label_set_text(dial_label_bottom, "Preset Select");

    // Initial display
    update_center_label(current_preset, COLOR_ACTIVE);
}

void preset_page_on_enter(void)
{
    current_preset = helix_get_current_preset();
    pending_preset = current_preset;
    has_pending    = false;

    update_center_label(current_preset, COLOR_ACTIVE);
}

void preset_page_delta(int delta)
{
    int next = (int)pending_preset + delta;

    if (next < 0)  next = 0;
    if (next > 7)  next = 7;

    if ((uint8_t)next == pending_preset)
        return;

    pending_preset = (uint8_t)next;
    has_pending = true;

    update_center_label(pending_preset, COLOR_ACTIVE);
}

void preset_page_select(void)
{
    if (!has_pending)
        return;

    Serial.printf(
        "[UI] Preset select request: %u\n",
        pending_preset + 1
    );

    helix_select_preset(pending_preset);
}

void preset_page_refresh(void)
{
    update_center_label(current_preset, COLOR_ACTIVE);
}

// ================= PROTOCOL CALLBACK =================
void preset_page_on_ack(uint8_t idx)
{
    current_preset = idx;
    pending_preset = idx;
    has_pending = false;

    flash_selected_preset();
}
