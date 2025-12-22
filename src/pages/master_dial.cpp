#include "master_dial.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>

// ================= INTERNAL UI OBJECTS =================

static lv_obj_t* dial_arc      = nullptr;
static lv_obj_t* dial_label    = nullptr;
static lv_obj_t* dial_function = nullptr;

// ================= DEFAULT STYLING =================

static const lv_color_t DIAL_BG_COLOR   = lv_color_hex(0x000000);
static const lv_color_t DIAL_FONT_COLOR = lv_color_hex(0xFFFFFF);

// ================= PER-SLOT STATE =================

typedef struct {
    lv_color_t color;
    const char* label;
} DialSlotState;

static DialSlotState slots[DIAL_SLOT_COUNT] = {
    { lv_color_hex(0x44CC44), "MASTER\nVOLUME" },   // VOL1
    { lv_color_hex(0xCC4444), "SUB\nLEVEL"    },   // VOL2
    { lv_color_hex(0x4488CC), "DIGITAL\nLEVEL"},   // VOL3
    { lv_color_hex(0xCCCC44), "REAR\nLEVEL"   }    // VOL4
};

// ================= ACTIVE STATE =================

static DialSlot active_slot = DIAL_SLOT_VOL1;
static int dial_value = 0;

// ================= INTERNAL HELPERS =================

static void dial_apply_color()
{
    if (!dial_arc)
        return;

    lv_obj_set_style_arc_color(
        dial_arc,
        slots[active_slot].color,
        LV_PART_INDICATOR
    );
}

static void dial_update()
{
    if (!dial_arc || !dial_label)
        return;

    lv_arc_set_value(dial_arc, dial_value);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", dial_value);
    lv_label_set_text(dial_label, buf);
}

// ================= PUBLIC API =================

void master_dial_create(lv_obj_t* parent)
{
    // Background
    lv_obj_set_style_bg_color(parent, DIAL_BG_COLOR, 0);

    // ----- ARC -----
    dial_arc = lv_arc_create(parent);
    lv_obj_set_size(dial_arc, 220, 220);
    lv_obj_center(dial_arc);

    lv_obj_remove_style(dial_arc, nullptr, LV_PART_KNOB);
    lv_obj_set_style_arc_rounded(dial_arc, false, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(dial_arc, false, LV_PART_INDICATOR);

    lv_obj_set_style_arc_width(dial_arc, 24, LV_PART_MAIN);
    lv_obj_set_style_arc_width(dial_arc, 24, LV_PART_INDICATOR);

    lv_arc_set_bg_start_angle(dial_arc, 145);
    lv_arc_set_bg_end_angle(dial_arc, 35);
    lv_arc_set_start_angle(dial_arc, 145);
    lv_arc_set_end_angle(dial_arc, 35);

    lv_arc_set_range(dial_arc, 0, 100);

    lv_obj_set_style_arc_color(
        dial_arc,
        lv_color_hex(0x333333),
        LV_PART_MAIN
    );

    dial_apply_color();

    // ----- CENTER LABEL -----
    dial_label = lv_label_create(parent);
    lv_obj_center(dial_label);
    lv_obj_set_style_text_font(dial_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(dial_label, DIAL_FONT_COLOR, 0);

    // ----- FUNCTION LABEL -----
    dial_function = lv_label_create(parent);
    lv_obj_set_style_text_font(dial_function, &lv_font_montserrat_20, 0);
    lv_obj_align(dial_function, LV_ALIGN_BOTTOM_MID, 0, -35);
    lv_label_set_text(dial_function, slots[active_slot].label);
    lv_obj_set_style_text_color(dial_function, DIAL_FONT_COLOR, 0);

    // Initial state
    dial_value = 0;
    dial_update();
}

void master_dial_set_absolute(int value)
{
    dial_value = value;

    if (dial_value < 0)   dial_value = 0;
    if (dial_value > 100) dial_value = 100;

    dial_update();

    Serial.printf("[UI] Dial absolute = %d\n", dial_value);
}

void master_dial_set_delta(int delta)
{
    master_dial_set_absolute(dial_value + delta);
}

int master_dial_get_value()
{
    return dial_value;
}

// ================= NEW SLOT / COLOR API =================

void master_dial_set_slot(DialSlot slot)
{
    if (slot >= DIAL_SLOT_COUNT)
        return;

    active_slot = (DialSlot)slot;

    if (dial_function)
        lv_label_set_text(dial_function, slots[active_slot].label);

    dial_apply_color();

    Serial.printf("[UI] Active dial slot = %u\n", slot);
}

void master_dial_set_color(DialSlot slot, uint8_t r, uint8_t g, uint8_t b)
{
    if (slot >= DIAL_SLOT_COUNT)
        return;

    slots[slot].color = lv_color_make(r, g, b);

    if (slot == active_slot)
        dial_apply_color();

    Serial.printf(
        "[UI] Slot %u color = #%02X%02X%02X\n",
        slot, r, g, b
    );
}
