#include <Arduino.h>
#include "master_dial.h"
#include <lvgl.h>
#include <cstdio>
#include "protocol/helix_protocol.h"

// ---------------- Internal State (private to this file) ----------------
static lv_obj_t* dial_arc;
static lv_obj_t* dial_label;
static lv_obj_t* dial_function;
static const lv_color_t DIAL_BG_COLOR        = lv_color_hex(0x000000);  // Black
static const lv_color_t DIAL_ARC_MAIN_COLOR  = lv_color_hex(0xCCCCCC);  // Light gray
static const lv_color_t DIAL_ARC_IND_COLOR   = lv_color_hex(0x44CC44);  // Green
static const lv_color_t DIAL_FONT_COLOR      = lv_color_hex(0xFFFFFF);  // White
static int dial_value = 50;

// ---------------- Internal Helper ----------------
static inline void dial_update_label(int dial_value) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", dial_value);
    lv_label_set_text(dial_label, buf);
}


// ---------------- Public API Implementations ----------------
void master_dial_create(lv_obj_t* parent)
{
    // ----- DIAL ----
    lv_obj_set_style_bg_color(parent, DIAL_BG_COLOR, 0);

    // ----- ARC -----
    dial_arc = lv_arc_create(parent);
    lv_obj_set_size(dial_arc, 220, 220);
    lv_obj_center(dial_arc);

    lv_obj_remove_style(dial_arc, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_rounded(dial_arc, false, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(dial_arc, false, LV_PART_INDICATOR);

    lv_obj_set_style_arc_width(dial_arc, 24, LV_PART_MAIN);
    lv_obj_set_style_arc_width(dial_arc, 24, LV_PART_INDICATOR);

    lv_arc_set_bg_start_angle(dial_arc, 145);
    lv_arc_set_bg_end_angle(dial_arc, 35);
    lv_arc_set_start_angle(dial_arc, 145);
    lv_arc_set_end_angle(dial_arc, 35);

    lv_obj_set_style_arc_color(dial_arc, DIAL_ARC_MAIN_COLOR, LV_PART_MAIN);
    lv_obj_set_style_arc_color(dial_arc, DIAL_ARC_IND_COLOR, LV_PART_INDICATOR);

    // ----- CENTER LABEL -----
    dial_label = lv_label_create(parent);
    lv_obj_center(dial_label);
    lv_obj_set_style_text_font(dial_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(dial_label, DIAL_FONT_COLOR, 0);

    // ----- FUNCTION LABEL -----
    dial_function = lv_label_create(parent);
    lv_obj_set_style_text_font(dial_function, &lv_font_montserrat_20, 0);
    lv_obj_align(dial_function, LV_ALIGN_BOTTOM_MID, 0, -35);
    lv_label_set_text(dial_function, "MASTER\nVOLUME");
    lv_obj_set_style_text_color(dial_function, DIAL_FONT_COLOR, 0);

    // Initial state
    lv_arc_set_value(dial_arc, dial_value);
    dial_update_label(dial_value);
}

void master_dial_set_value(int delta)
{
    // UI optimism: move visually immediately
    dial_value += delta;
    if (dial_value < 0)   dial_value = 0;
    if (dial_value > 100) dial_value = 100;

    lv_arc_set_value(dial_arc, dial_value);
    dial_update_label(dial_value);

    // Protocol intent
    helix_volume_delta(delta);

    Serial.printf("Master Dial: %d\n", dial_value);

}

int master_dial_get_value()
{
    return dial_value;
}
