#include "preset_page.h"
#include <Arduino.h>
#include <lvgl.h>
#include "src/misc/lv_timer.h"
#include <cstdio>
#include "protocol/helix_protocol.h"
#include "fonts/lv_font_montserrat_64.h"

// ================= INTERNAL UI OBJECTS =================

static lv_obj_t* dial_label_center = nullptr;
static lv_obj_t* dial_label_bottom = nullptr;
static lv_obj_t* preset_page_root  = nullptr;
static lv_timer_t* no_config_timer = nullptr;
static lv_timer_t* bg_flash_timer  = nullptr;


// ================= STYLING =================

static const lv_color_t BG_COLOR = lv_color_hex(0x000000);
static constexpr int DIGIT_PAD_ADJUST = 25;   // positive = move DOWN

// ================= STATE =================

static uint8_t current_preset        = 0;   // UI-confirmed active
static uint8_t pending_preset        = 0;   // dial-selected
static uint8_t last_committed_preset = 0;   // last DSP-accepted
static bool    has_pending           = false;

// ================= INTERNAL HELPERS =================

static void update_center_label(uint8_t idx)
{
    if (!dial_label_center)
        return;

    char buf[4];
    snprintf(buf, sizeof(buf), "%u", idx + 1);

    lv_label_set_text(dial_label_center, buf);
}

static void apply_digit_padding(void)
{
    lv_obj_set_style_pad_top(
        dial_label_center,
        (lv_font_montserrat_64.line_height / 2)
            - lv_font_montserrat_64.base_line
            - 4
            + DIGIT_PAD_ADJUST,
        0
    );
}

static void apply_no_config_padding(void)
{
    lv_obj_set_style_pad_top(
        dial_label_center,
        4,   // whatever value you tuned for No Config
        0
    );
}

static void bg_flash_cb(lv_timer_t*)
{
    lv_obj_set_style_bg_color(
        preset_page_root,
        BG_COLOR,
        0
    );

    bg_flash_timer = nullptr;
}

static void flash_selected_preset(void)
{
    if (!preset_page_root)
        return;

    if (bg_flash_timer) {
        lv_timer_del(bg_flash_timer);
        bg_flash_timer = nullptr;
    }

    lv_obj_set_style_bg_color(
        preset_page_root,
        lv_color_hex(0xDDDDDD),
        0
    );

    bg_flash_timer = lv_timer_create(
        bg_flash_cb,
        150,
        nullptr
    );

    lv_timer_set_repeat_count(bg_flash_timer, 1);
}

static void no_config_cb(lv_timer_t*)
{
    update_center_label(last_committed_preset);
    apply_digit_padding();
    lv_obj_center(dial_label_center);

    no_config_timer = nullptr;
}

static void flash_no_config(void)
{
    if (!dial_label_center)
        return;

    if (no_config_timer) {
        lv_timer_del(no_config_timer);
        no_config_timer = nullptr;
    }

    apply_no_config_padding();
    lv_label_set_text(dial_label_center, "No\nConfig");
    lv_obj_center(dial_label_center);

    no_config_timer = lv_timer_create(
        no_config_cb,
        1000,
        nullptr
    );

    lv_timer_set_repeat_count(no_config_timer, 1);
}

// ================= PUBLIC API =================

void preset_page_create(lv_obj_t* parent)
{
    preset_page_root = parent;

    lv_obj_set_style_bg_color(parent, BG_COLOR, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    // ----- CENTER LABEL (FORCED DEBUG) -----
    dial_label_center = lv_label_create(parent);

    lv_obj_set_style_text_font(
        dial_label_center,
        &lv_font_montserrat_64,
        0
    );

    lv_obj_set_style_text_color(
        dial_label_center,
        lv_color_hex(0xFFFFFF),
        0
    );

    lv_obj_set_style_text_align(
        dial_label_center,
        LV_TEXT_ALIGN_CENTER,
        0
    );

    lv_obj_set_size(
        dial_label_center,
        240,
        (lv_font_montserrat_64.line_height * 2)+ 8
    );

    lv_obj_set_style_pad_top(
        dial_label_center,
        (lv_font_montserrat_64.line_height / 2)
            - lv_font_montserrat_64.base_line
            - 4
            + DIGIT_PAD_ADJUST,
        0
    );

    lv_label_set_long_mode(
        dial_label_center,
        LV_LABEL_LONG_WRAP
    );

    apply_digit_padding();
    update_center_label(current_preset);
    lv_obj_center(dial_label_center);

    // ----- BOTTOM LABEL -----
    dial_label_bottom = lv_label_create(parent);

    lv_obj_set_style_text_font(
        dial_label_bottom,
        &lv_font_montserrat_20,
        0
    );

    lv_obj_set_style_text_color(
        dial_label_bottom,
        lv_color_hex(0xFFFFFF),
        0
    );

    lv_label_set_text(dial_label_bottom, "Preset Select");
    lv_obj_align(dial_label_bottom, LV_ALIGN_BOTTOM_MID, 0, -35);
}

void preset_page_on_enter(void)
{
    if (no_config_timer) {
        lv_timer_del(no_config_timer);
        no_config_timer = nullptr;
    }
    if (bg_flash_timer) {
        lv_timer_del(bg_flash_timer);
        bg_flash_timer = nullptr;
    }

    lv_obj_invalidate(lv_scr_act());

    current_preset        = helix_get_current_preset();
    pending_preset        = current_preset;
    last_committed_preset = current_preset;
    has_pending           = false;

    apply_digit_padding();
    update_center_label(current_preset);
}

void preset_page_delta(int delta)
{
    int next = (int)pending_preset + delta;

    if (next < 0) next = 0;
    if (next > 9) next = 9;

    if (next == pending_preset)
        return;

    pending_preset = (uint8_t)next;
    has_pending = true;

    update_center_label(pending_preset);
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
    // Sync with protocol state in case preset was updated during handshake
    current_preset = helix_get_current_preset();
    update_center_label(current_preset);
}

// ================= PROTOCOL CALLBACK =================
void preset_page_on_ack(uint8_t idx, uint8_t status)
{
    Serial.printf(
        "[PRESET ACK] idx=%u status=%02X\n",
        idx + 1,
        status
    );

    if (status == 0x00) {
        has_pending = false;
        flash_no_config();
        return;
    }

    current_preset        = idx;
    pending_preset        = idx;
    last_committed_preset = idx;
    has_pending           = false;

    apply_digit_padding();
    update_center_label(idx);
    lv_obj_center(dial_label_center);

    flash_selected_preset();
}
