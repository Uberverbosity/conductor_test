#include "volume_page.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>
#include <cmath>
#include "protocol/helix_protocol.h"
#include "settings.h"

// ================= INTERNAL UI OBJECTS =================

static lv_obj_t* dial_arc      = nullptr;
static lv_obj_t* dial_label    = nullptr;
static lv_obj_t* dial_function = nullptr;

static bool use_mcintosh = false;
static lv_obj_t* mc_bg = nullptr;
static lv_obj_t* mc_bottom = nullptr;
static lv_obj_t* mc_outer_ring = nullptr;
static lv_obj_t* mc_shroud = nullptr;
static lv_obj_t* mc_shroud_hump_border = nullptr;
static lv_obj_t* mc_shroud_hump = nullptr;
static lv_obj_t* mc_shroud_border = nullptr;
static lv_obj_t* mc_shroud_edge = nullptr;
static lv_obj_t* mc_glow = nullptr;
static lv_obj_t* mc_arc_main = nullptr;
static lv_obj_t* mc_arc_hot = nullptr;
static lv_obj_t* mc_needle = nullptr;
static lv_obj_t* mc_center_dot = nullptr;
static lv_obj_t* mc_value_label = nullptr;
static lv_obj_t* mc_ticks[13] = {nullptr};

// ================= DEFAULT STYLING =================

static const lv_color_t DIAL_BG_COLOR   = lv_color_hex(0x000000);
static const lv_color_t DIAL_FONT_COLOR = lv_color_hex(0xFFFFFF);

static const lv_color_t MC_BLUE_BG      = lv_color_hex(0x0E4FB8);
static const lv_color_t MC_BLUE_GLOW    = lv_color_hex(0x2A7BD8);
static const lv_color_t MC_ARC_BLUE     = lv_color_hex(0x93D7FF);
static const lv_color_t MC_ARC_RED      = lv_color_hex(0xFF5533);
static const lv_color_t MC_NEEDLE_COLOR = lv_color_hex(0xFFFFFF);
static const lv_color_t MC_TEXT_COLOR   = lv_color_hex(0xFFFFFF);
static const lv_color_t MC_BORDER_GREY  = lv_color_hex(0x1C1C1C);

// ================= PER-SLOT STATE =================

typedef struct {
    lv_color_t color;
} DialSlotState;

static DialSlotState slots[DIAL_SLOT_COUNT];

// ================= ACTIVE STATE =================

static DialSlot active_slot = DIAL_SLOT_VOL1;
static int dial_value = 0;

// ================= INTERNAL HELPERS =================

static void dial_apply_color()
{
    if (use_mcintosh) {
        if (!mc_needle)
            return;

        lv_obj_set_style_line_color(
            mc_needle,
            slots[active_slot].color,
            0
        );
        return;
    }

    if (!dial_arc)
        return;

    lv_obj_set_style_arc_color(
        dial_arc,
        slots[active_slot].color,
        LV_PART_INDICATOR
    );
}

static void mcintosh_update()
{
    if (!mc_needle)
        return;

    const int center_x = 120;
    const int center_y = 170;
    const int radius = 92;

    const int arc_start = 145;
    const int arc_end = 35;
    const int arc_span = (360 - arc_start) + arc_end; // 250
    int angle = arc_start + (arc_span * dial_value) / 100;
    if (angle >= 360) angle -= 360;

    const float kDegToRad = 0.0174532925f;
    float rad = angle * kDegToRad;

    static lv_point_precise_t needle_points[2];
    needle_points[0].x = center_x;
    needle_points[0].y = center_y;
    needle_points[1].x = (lv_coord_t)(center_x + std::cos(rad) * radius);
    needle_points[1].y = (lv_coord_t)(center_y + std::sin(rad) * radius);

    lv_line_set_points(mc_needle, needle_points, 2);

    if (mc_value_label) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", dial_value);
        lv_label_set_text(mc_value_label, buf);
    }
}

static void dial_update()
{
    if (use_mcintosh) {
        mcintosh_update();
        return;
    }

    if (!dial_arc || !dial_label)
        return;

    lv_arc_set_value(dial_arc, dial_value);

    char buf[8];
    snprintf(buf, sizeof(buf), "%d", dial_value);
    lv_label_set_text(dial_label, buf);
}

static void mcintosh_create(lv_obj_t* parent)
{
    lv_obj_set_style_bg_color(parent, DIAL_BG_COLOR, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);

    mc_bg = lv_obj_create(parent);
    lv_obj_set_size(mc_bg, 240, 240);
    lv_obj_center(mc_bg);
    lv_obj_set_style_bg_color(mc_bg, MC_BLUE_BG, 0);
    lv_obj_set_style_bg_grad_color(mc_bg, MC_BLUE_GLOW, 0);
    lv_obj_set_style_bg_grad_dir(mc_bg, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(mc_bg, 0, 0);
    lv_obj_set_style_radius(mc_bg, 120, 0);

    mc_glow = lv_obj_create(mc_bg);
    lv_obj_set_size(mc_glow, 180, 120);
    lv_obj_align(mc_glow, LV_ALIGN_BOTTOM_MID, 0, 8);
    lv_obj_set_style_bg_color(mc_glow, MC_BLUE_GLOW, 0);
    lv_obj_set_style_bg_opa(mc_glow, LV_OPA_60, 0);
    lv_obj_set_style_border_width(mc_glow, 0, 0);
    lv_obj_set_style_radius(mc_glow, 90, 0);

    mc_arc_main = lv_arc_create(mc_bg);
    lv_obj_set_size(mc_arc_main, 210, 210);
    lv_obj_center(mc_arc_main);
    lv_obj_remove_style(mc_arc_main, nullptr, LV_PART_KNOB);
    lv_obj_set_style_arc_width(mc_arc_main, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(mc_arc_main, MC_ARC_BLUE, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(mc_arc_main, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(mc_arc_main, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_arc_set_bg_start_angle(mc_arc_main, 145);
    lv_arc_set_bg_end_angle(mc_arc_main, 35);
    lv_arc_set_start_angle(mc_arc_main, 145);
    lv_arc_set_end_angle(mc_arc_main, 35);
    lv_arc_set_range(mc_arc_main, 0, 100);

    mc_arc_hot = lv_arc_create(mc_bg);
    lv_obj_set_size(mc_arc_hot, 210, 210);
    lv_obj_center(mc_arc_hot);
    lv_obj_remove_style(mc_arc_hot, nullptr, LV_PART_KNOB);
    lv_obj_set_style_arc_width(mc_arc_hot, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(mc_arc_hot, MC_ARC_RED, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(mc_arc_hot, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(mc_arc_hot, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_arc_set_bg_start_angle(mc_arc_hot, 300);
    lv_arc_set_bg_end_angle(mc_arc_hot, 35);
    lv_arc_set_start_angle(mc_arc_hot, 300);
    lv_arc_set_end_angle(mc_arc_hot, 35);
    lv_arc_set_range(mc_arc_hot, 0, 100);

    for (int i = 0; i < 13; i++) {
        mc_ticks[i] = lv_line_create(mc_bg);
        lv_obj_set_style_line_width(mc_ticks[i], (i % 3 == 0) ? 3 : 2, 0);
        lv_obj_set_style_line_color(mc_ticks[i], MC_TEXT_COLOR, 0);
    }

    const int center_x = 120;
    const int center_y = 120;
    const int outer_r = 98;
    const int inner_r = 84;
    const int arc_start = 145;
    const int arc_end = 35;
    const int arc_span = (360 - arc_start) + arc_end;
    const float kDegToRad = 0.0174532925f;

    for (int i = 0; i < 13; i++) {
        int angle = arc_start + (arc_span * i) / 12;
        if (angle >= 360) angle -= 360;

        float rad = angle * kDegToRad;
        int r_inner = (i % 3 == 0) ? inner_r - 6 : inner_r;
        int r_outer = outer_r;

        static lv_point_precise_t pts[2];
        pts[0].x = (lv_coord_t)(center_x + std::cos(rad) * r_inner);
        pts[0].y = (lv_coord_t)(center_y + std::sin(rad) * r_inner);
        pts[1].x = (lv_coord_t)(center_x + std::cos(rad) * r_outer);
        pts[1].y = (lv_coord_t)(center_y + std::sin(rad) * r_outer);
        lv_line_set_points(mc_ticks[i], pts, 2);
    }

    mc_needle = lv_line_create(mc_bg);
    lv_obj_set_style_line_width(mc_needle, 3, 0);
    lv_obj_set_style_line_color(mc_needle, MC_NEEDLE_COLOR, 0);

    // mc_center_dot = lv_obj_create(mc_bg);
    // lv_obj_set_size(mc_center_dot, 10, 10);
    // lv_obj_align(mc_center_dot, LV_ALIGN_CENTER, 0, 50);
    // lv_obj_set_style_bg_color(mc_center_dot, MC_NEEDLE_COLOR, 0);
    // lv_obj_set_style_radius(mc_center_dot, 5, 0);
    // lv_obj_set_style_border_width(mc_center_dot, 0, 0);

    mc_shroud_border = lv_obj_create(parent);
    lv_obj_set_size(mc_shroud_border, 239, 8);
    lv_obj_align(mc_shroud_border, LV_ALIGN_BOTTOM_LEFT, 0, -87); // top edge line
    lv_obj_set_style_bg_color(mc_shroud_border, MC_BORDER_GREY, 0);
    lv_obj_set_style_border_width(mc_shroud_border, 0, 0);
    lv_obj_set_style_radius(mc_shroud_border, 0, 0);
    lv_obj_clear_flag(mc_shroud_border, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(mc_shroud_border, LV_SCROLLBAR_MODE_OFF);

    mc_shroud_hump_border = lv_obj_create(parent);
    lv_obj_set_size(mc_shroud_hump_border, 200, 200);
    lv_obj_align(mc_shroud_hump_border, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_color(mc_shroud_hump_border, MC_BORDER_GREY, 0);
    lv_obj_set_style_border_width(mc_shroud_hump_border, 0, 0);
    lv_obj_set_style_radius(mc_shroud_hump_border, 100, 0);

    mc_shroud = lv_obj_create(parent);
    lv_obj_set_size(mc_shroud, 239, 100);
    lv_obj_align(mc_shroud, LV_ALIGN_BOTTOM_LEFT, 0, 13); //vertical offset
    lv_obj_set_style_bg_color(mc_shroud, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(mc_shroud, 0, 0);
    lv_obj_set_style_radius(mc_shroud, 0, 0);
    lv_obj_clear_flag(mc_shroud, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(mc_shroud, LV_SCROLLBAR_MODE_OFF);

    mc_shroud_hump = lv_obj_create(parent);
    lv_obj_set_size(mc_shroud_hump, 184, 184);
    lv_obj_align(mc_shroud_hump, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_color(mc_shroud_hump, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(mc_shroud_hump, 0, 0);
    lv_obj_set_style_radius(mc_shroud_hump, 92, 0);

    mc_shroud_edge = nullptr;

    mc_bottom = lv_obj_create(parent);
    lv_obj_set_size(mc_bottom, 240, 70);
    lv_obj_align(mc_bottom, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(mc_bottom, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_bottom, 0, 0);

    dial_function = lv_label_create(mc_bottom);
    lv_obj_set_style_text_font(dial_function, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(dial_function, MC_TEXT_COLOR, 0);
    lv_obj_center(dial_function);

    mc_value_label = lv_label_create(mc_bg);
    lv_obj_set_style_text_font(mc_value_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(mc_value_label, MC_TEXT_COLOR, 0);
    lv_obj_align(mc_value_label, LV_ALIGN_CENTER, 0, 10);

    mc_outer_ring = lv_obj_create(parent);
    lv_obj_set_size(mc_outer_ring, 240, 240);
    lv_obj_center(mc_outer_ring);
    lv_obj_set_style_bg_opa(mc_outer_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_outer_ring, 8, 0);
    lv_obj_set_style_border_color(mc_outer_ring, MC_BORDER_GREY, 0);
    lv_obj_set_style_radius(mc_outer_ring, 120, 0);

    dial_value = 0;
    dial_apply_color();
    mcintosh_update();
}

// ================= PUBLIC API =================

void volume_page_create(lv_obj_t* parent)
{
    use_mcintosh = (settings_get_theme() == THEME_MCINTOSH);

    if (use_mcintosh) {
        mcintosh_create(parent);
        return;
    }

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
    lv_obj_set_style_text_color(dial_function, DIAL_FONT_COLOR, 0);

    // Initial state
    dial_value = 0;
    dial_update();
}

void volume_page_set_absolute(int value)
{
    dial_value = value;

    if (dial_value < 0)   dial_value = 0;
    if (dial_value > 100) dial_value = 100;

    dial_update();

}

void volume_page_set_delta(int delta)
{
    volume_page_set_absolute(dial_value + delta);
}

int volume_page_get_value()
{
    return dial_value;
}

// ================= NEW SLOT / COLOR API =================

void volume_page_set_slot(DialSlot slot)
{
    if (slot >= DIAL_SLOT_COUNT)
        return;

    active_slot = (DialSlot)slot;

    dial_apply_color();

}

void volume_page_set_color(DialSlot slot, uint8_t r, uint8_t g, uint8_t b)
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

void volume_page_set_label(const char* text)
{
    if (!dial_function)
        return;

    lv_label_set_text(dial_function, text ? text : "");
}

void volume_page_on_enter()
{
    // Re-assert current slot context
    DialSlot slot = helix_get_active_slot();

    volume_page_set_slot(slot);

    // Re-assert label (assign name already comes from protocol)
    volume_page_set_label(
        helix_get_slot_label(slot)
    );

    // Re-assert value (absolute, authoritative)
    int ui = helix_get_slot_ui_value(slot);
    volume_page_set_absolute(ui);
}

void volume_page_delta(int delta)
{
    helix_volume_delta(delta);
}

void volume_page_refresh()
{
    DialSlot slot = helix_get_active_slot();

    int ui = helix_get_slot_ui_value(slot);
    const char* label = helix_get_slot_label(slot);

    volume_page_set_label(label);
    volume_page_set_absolute(ui);
}
