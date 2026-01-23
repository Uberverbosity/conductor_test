#include "volume_page.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>
#include <cmath>
#include "protocol/helix_protocol.h"
#include "settings.h"
#include "../images/blue_background.h"

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
static const lv_color_t MC_NEEDLE_COLOR = lv_color_hex(0x000000);
static const lv_color_t MC_TEXT_COLOR   = lv_color_hex(0xFFFFFF);
static const lv_color_t MC_BORDER_GREY  = lv_color_hex(0x161616);

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

        // McIntosh needles are always black, regardless of slot color
        lv_obj_set_style_line_color(
            mc_needle,
            MC_NEEDLE_COLOR,
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

    const int center_x = 108;  // Center of 240px wide display/container
    const int center_y = 217;  // 23 pixels up from bottom (240 - 23 = 217)
    const float radius = 170.0f;  // Fixed needle length

    // Needle sweeps from 0 mark (-45° from 12 o'clock) to 100 mark (+45° from 12 o'clock)
    // In LVGL: 0° = 3 o'clock, 90° = 6 o'clock, 180° = 9 o'clock, 270° = 12 o'clock
    // Converting from 12 o'clock reference: -45° = 270° - 45° = 225°, +45° = 270° + 45° = 315°
    const int angle_at_0 = 244;   // -45° from 12 o'clock (8 o'clock position)
    const int angle_at_100 = 296; // +45° from 12 o'clock (4 o'clock position)
    const int arc_span = angle_at_100 - angle_at_0; // 90 degrees total span
    
    // Calculate needle angle: 0 value = angle_at_0, 100 value = angle_at_100
    int angle = angle_at_0 + (arc_span * dial_value) / 100;
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

// Note: Gradient image is now loaded from blue_background.cpp file
// The create_gradient_image() function has been removed in favor of the pre-generated image

static void mcintosh_create(lv_obj_t* parent)
{
    lv_obj_set_style_bg_color(parent, DIAL_BG_COLOR, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);

    // Use pre-generated smooth radial gradient image from blue_volume.cpp
    // This creates a perfect smooth gradient that would be difficult with layered objects
    const lv_img_dsc_t* gradient_img = get_blue_volume();
    
    // Create image widget to display the gradient background
    lv_obj_t* gradient_bg = lv_img_create(parent);
    lv_img_set_src(gradient_bg, gradient_img);
    lv_obj_set_size(gradient_bg, 240, 240);
    lv_obj_center(gradient_bg);
    lv_obj_set_style_radius(gradient_bg, 120, 0);
    lv_obj_set_style_clip_corner(gradient_bg, true, 0);
    
    // Container for UI elements (transparent, sits on top of gradient image)
    mc_bg = lv_obj_create(parent);
    lv_obj_set_size(mc_bg, 240, 240);
    lv_obj_center(mc_bg);
    lv_obj_set_style_bg_opa(mc_bg, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_bg, 0, 0);
    lv_obj_set_style_radius(mc_bg, 120, 0);

    mc_needle = lv_line_create(mc_bg);
    lv_obj_set_style_line_width(mc_needle, 3, 0);
    lv_obj_set_style_line_color(mc_needle, MC_NEEDLE_COLOR, 0);

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
    lv_obj_set_size(mc_bottom, 240, 120);
    lv_obj_align(mc_bottom, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(mc_bottom, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_bottom, 0, 0);

    dial_function = lv_label_create(mc_bottom);
    lv_obj_set_style_text_font(dial_function, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(dial_function, MC_TEXT_COLOR, 0);
    lv_obj_set_style_text_align(dial_function, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(dial_function, LV_ALIGN_CENTER, 0, 0);

    mc_value_label = lv_label_create(mc_bg);
    lv_obj_set_style_text_font(mc_value_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(mc_value_label, MC_TEXT_COLOR, 0);
    lv_obj_set_style_text_align(mc_value_label, LV_TEXT_ALIGN_CENTER, 0);
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
    lv_obj_set_style_text_font(dial_function, &lv_font_montserrat_24, 0);
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
