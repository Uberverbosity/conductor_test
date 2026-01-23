#include "tone_page.h"
#include "protocol/helix_protocol.h"
#include <lvgl.h>
#include <cstdio>
#include <cmath>
#include "settings.h"
#include "../images/blue_background.h"

// ================= GEOMETRY =================

// Match volume_page exactly
#define ARC_LEFT       145
#define ARC_RIGHT       35
#define ARC_SPAN       ((360 - ARC_LEFT) + ARC_RIGHT)   // 250°
#define ARC_CENTER     ((ARC_LEFT + ARC_SPAN / 2) % 360) // 270° (12 o'clock)

#define UNITY_TICK_DEG  6   // small mark at 0 dB

// ================= INTERNAL UI OBJECTS =================

struct TonePageCtx {
    lv_obj_t* arc;
    lv_obj_t* label_center;
    lv_obj_t* label_frequency;
    lv_obj_t* label_bottom;
};

static TonePageCtx ctx;
static ToneBand s_band;
static bool use_mcintosh = false;

// McIntosh UI objects
static lv_obj_t* mc_tone_bg = nullptr;
static lv_obj_t* mc_tone_needle = nullptr;
static lv_obj_t* mc_tone_bottom = nullptr;
static lv_obj_t* mc_tone_outer_ring = nullptr;
static lv_obj_t* mc_tone_shroud = nullptr;
static lv_obj_t* mc_tone_shroud_hump_border = nullptr;
static lv_obj_t* mc_tone_shroud_hump = nullptr;
static lv_obj_t* mc_tone_shroud_border = nullptr;
static lv_obj_t* mc_tone_label_center = nullptr;
static lv_obj_t* mc_tone_label_frequency = nullptr;
static lv_obj_t* mc_tone_label_bottom = nullptr;

// ================= DEFAULT STYLING =================

static const lv_color_t DIAL_BG_COLOR   = lv_color_hex(0x000000);
static const lv_color_t DIAL_FONT_COLOR = lv_color_hex(0xFFFFFF);
static const lv_color_t MC_NEEDLE_COLOR = lv_color_hex(0x000000);
static const lv_color_t MC_TEXT_COLOR   = lv_color_hex(0xFFFFFF);
static const lv_color_t MC_BORDER_GREY  = lv_color_hex(0x161616);

// ================= ARC COLORS =====================
static lv_color_t tone_arc_color(ToneBand band)
{
    return (band == TONE_LOW)
        ? lv_color_hex(0x7B3FE4)   // Purple
        : lv_color_hex(0xFF8C1A);  // Orange
}

// ================= INTERNAL HELPERS =================

static void mcintosh_tone_update(void)
{
    if (!mc_tone_needle)
        return;

    bool valid;
    int8_t db;

    if (s_band == TONE_LOW) {
        valid = helix_tone_low_valid();
        db    = helix_tone_low_db();
    } else {
        valid = helix_tone_high_valid();
        db    = helix_tone_high_db();
    }

    if (!valid) {
        // Hide needle or set to center when invalid
        db = 0;
    }

    // Clamp dB to valid range
    if (db < -12) db = -12;
    if (db > 12) db = 12;

    // Needle positioning: same pivot and length as volume page
    const int center_x = 108;  // Center of 240px wide display/container
    const int center_y = 217;  // 23 pixels up from bottom (240 - 23 = 217)
    const float radius = 170.0f;  // Fixed needle length

    // Needle sweeps from -12 dB to +12 dB centered at 0 dB (12 o'clock = 270°)
    // Tone page: -12 dB = 230°, 0 dB = 270°, +12 dB = 310°
    const int angle_at_minus12 = 240;
    const int angle_at_center = 270;
    const int angle_at_plus12 = 299;
    
    // Calculate spans dynamically from the angle constants
    const int span_negative = angle_at_center - angle_at_minus12;  // Degrees from center to -12 dB
    const int span_positive = angle_at_plus12 - angle_at_center;   // Degrees from center to +12 dB

    int angle;
    if (db == 0) {
        angle = angle_at_center;
    } else if (db > 0) {
        // Positive dB: linear interpolation from center to +12 dB
        float t = db / 12.0f;  // 0.0 to 1.0
        angle = angle_at_center + (int)(t * span_positive);
    } else {
        // Negative dB: linear interpolation from center to -12 dB
        float t = (-db) / 12.0f;  // 0.0 to 1.0
        angle = angle_at_center - (int)(t * span_negative);
    }

    if (angle >= 360) angle -= 360;
    if (angle < 0) angle += 360;

    const float kDegToRad = 0.0174532925f;
    float rad = angle * kDegToRad;

    static lv_point_precise_t needle_points[2];
    needle_points[0].x = center_x;
    needle_points[0].y = center_y;
    needle_points[1].x = (lv_coord_t)(center_x + std::cos(rad) * radius);
    needle_points[1].y = (lv_coord_t)(center_y + std::sin(rad) * radius);

    lv_line_set_points(mc_tone_needle, needle_points, 2);

    // Update labels
    if (mc_tone_label_center) {
        char buf[8];
        if (valid) {
            snprintf(buf, sizeof(buf), "%+d", db);
        } else {
            snprintf(buf, sizeof(buf), "--");
        }
        lv_label_set_text(mc_tone_label_center, buf);
    }

    if (mc_tone_label_frequency) {
        uint16_t freq;
        if (s_band == TONE_LOW) {
            freq = helix_tone_low_hz();
        } else {
            freq = helix_tone_high_hz();
        }
        char freq_buf[16];
        if (valid) {
            snprintf(freq_buf, sizeof(freq_buf), "%u Hz", freq);
        } else {
            freq_buf[0] = '\0';
        }
        lv_label_set_text(mc_tone_label_frequency, freq_buf);
    }
}

static void tone_page_set_db_internal(void)
{
    if (use_mcintosh) {
        mcintosh_tone_update();
        return;
    }

    // Basic theme: update arc and labels
    bool valid;
    int8_t db;
    uint16_t freq;

    if (s_band == TONE_LOW) {
        valid = helix_tone_low_valid();
        db    = helix_tone_low_db();
        freq  = helix_tone_low_hz();
    } else {
        valid = helix_tone_high_valid();
        db    = helix_tone_high_db();
        freq  = helix_tone_high_hz();
    }

    if (!valid) {
        lv_label_set_text(ctx.label_center, "--");
        lv_label_set_text(ctx.label_frequency, "");
        return;
    }

    // ----- CENTER LABEL -----
    char buf[8];
    snprintf(buf, sizeof(buf), "%+d", db);
    lv_label_set_text(ctx.label_center, buf);

    // ----- FREQUENCY LABEL -----
    char freq_buf[16];
    snprintf(freq_buf, sizeof(freq_buf), "%u Hz", freq);
    lv_label_set_text(ctx.label_frequency, freq_buf);

    // ----- ARC CALCULATION -----
    int start, end;

    constexpr int NEG_SPAN = ARC_CENTER - ARC_LEFT;          // 125°
    constexpr int POS_SPAN = (360 - ARC_CENTER) + ARC_RIGHT; // 125°

    if (db == 0) {
        start = ARC_CENTER - UNITY_TICK_DEG / 2;
        end   = ARC_CENTER + UNITY_TICK_DEG / 2;
    }
    else if (db > 0) {
        float t = db / 12.0f;          // 0 → 1
        int grow = (int)(t * POS_SPAN);

        start = ARC_CENTER;
        end   = ARC_CENTER + grow;
        if (end >= 360) end -= 360;
    }
    else {
        float t = (-db) / 12.0f;       // 0 → 1
        int grow = (int)(t * NEG_SPAN);

        start = ARC_CENTER - grow;
        if (start < 0) start += 360;
        end = ARC_CENTER;
    }

    lv_arc_set_start_angle(ctx.arc, start);
    lv_arc_set_end_angle(ctx.arc, end);
}

static void basic_tone_create(lv_obj_t* parent, ToneBand band)
{
    // Background
    lv_obj_set_style_bg_color(parent, DIAL_BG_COLOR, 0);

    // ----- ARC -----
    ctx.arc = lv_arc_create(parent);
    lv_obj_set_size(ctx.arc, 220, 220);
    lv_obj_center(ctx.arc);

    lv_obj_remove_style(ctx.arc, nullptr, LV_PART_KNOB);
    lv_obj_set_style_arc_rounded(ctx.arc, false, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(ctx.arc, false, LV_PART_INDICATOR);

    lv_obj_set_style_arc_width(ctx.arc, 24, LV_PART_MAIN);
    lv_obj_set_style_arc_width(ctx.arc, 24, LV_PART_INDICATOR);

    lv_arc_set_bg_start_angle(ctx.arc, ARC_LEFT);
    lv_arc_set_bg_end_angle(ctx.arc, ARC_RIGHT);
    lv_arc_set_start_angle(ctx.arc, ARC_LEFT);
    lv_arc_set_end_angle(ctx.arc, ARC_RIGHT);

    lv_obj_set_style_arc_color(
        ctx.arc,
        lv_color_hex(0x333333),
        LV_PART_MAIN
    );

    lv_obj_set_style_arc_color(
        ctx.arc,
        tone_arc_color(band),
        LV_PART_INDICATOR
    );

    // ----- CENTER LABEL -----
    ctx.label_center = lv_label_create(parent);
    lv_obj_center(ctx.label_center);
    lv_obj_set_style_text_font(
        ctx.label_center,
        &lv_font_montserrat_48,
        0
    );
    lv_obj_set_style_text_color(
        ctx.label_center,
        DIAL_FONT_COLOR,
        0
    );

    // ----- FREQUENCY LABEL -----
    ctx.label_frequency = lv_label_create(parent);
    lv_obj_set_style_text_font(
        ctx.label_frequency,
        &lv_font_montserrat_20,
        0
    );
    lv_obj_set_style_text_color(
        ctx.label_frequency,
        DIAL_FONT_COLOR,
        0
    );
    lv_obj_align(ctx.label_frequency, LV_ALIGN_BOTTOM_MID, 0, -70);

    // ----- BOTTOM LABEL -----
    ctx.label_bottom = lv_label_create(parent);
    lv_obj_set_style_text_font(
        ctx.label_bottom,
        &lv_font_montserrat_24,
        0
    );
    lv_obj_set_style_text_color(
        ctx.label_bottom,
        DIAL_FONT_COLOR,
        0
    );
    lv_obj_align(ctx.label_bottom, LV_ALIGN_BOTTOM_MID, 0, -35);

    lv_label_set_text(
        ctx.label_bottom,
        (band == TONE_LOW) ? "TONE LOW" : "TONE HIGH"
    );
}

static void mcintosh_tone_create(lv_obj_t* parent, ToneBand band)
{
    lv_obj_set_style_bg_color(parent, DIAL_BG_COLOR, 0);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);

    // Use pre-generated smooth radial gradient image from blue_tone.cpp
    const lv_img_dsc_t* gradient_img = get_blue_tone();
    
    // Create image widget to display the gradient background
    lv_obj_t* gradient_bg = lv_img_create(parent);
    lv_img_set_src(gradient_bg, gradient_img);
    lv_obj_set_size(gradient_bg, 240, 240);
    lv_obj_center(gradient_bg);
    lv_obj_set_style_radius(gradient_bg, 120, 0);
    lv_obj_set_style_clip_corner(gradient_bg, true, 0);
    
    // Container for UI elements (transparent, sits on top of gradient image)
    mc_tone_bg = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_bg, 240, 240);
    lv_obj_center(mc_tone_bg);
    lv_obj_set_style_bg_opa(mc_tone_bg, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_tone_bg, 0, 0);
    lv_obj_set_style_radius(mc_tone_bg, 120, 0);

    // Needle (no arcs/indicators in mcintosh theme)
    mc_tone_needle = lv_line_create(mc_tone_bg);
    lv_obj_set_style_line_width(mc_tone_needle, 3, 0);
    lv_obj_set_style_line_color(mc_tone_needle, MC_NEEDLE_COLOR, 0);

    // Shroud elements (same as volume page)
    mc_tone_shroud_border = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_shroud_border, 239, 8);
    lv_obj_align(mc_tone_shroud_border, LV_ALIGN_BOTTOM_LEFT, 0, -87);
    lv_obj_set_style_bg_color(mc_tone_shroud_border, MC_BORDER_GREY, 0);
    lv_obj_set_style_border_width(mc_tone_shroud_border, 0, 0);
    lv_obj_set_style_radius(mc_tone_shroud_border, 0, 0);
    lv_obj_clear_flag(mc_tone_shroud_border, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(mc_tone_shroud_border, LV_SCROLLBAR_MODE_OFF);

    mc_tone_shroud_hump_border = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_shroud_hump_border, 200, 200);
    lv_obj_align(mc_tone_shroud_hump_border, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_color(mc_tone_shroud_hump_border, MC_BORDER_GREY, 0);
    lv_obj_set_style_border_width(mc_tone_shroud_hump_border, 0, 0);
    lv_obj_set_style_radius(mc_tone_shroud_hump_border, 100, 0);

    mc_tone_shroud = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_shroud, 239, 100);
    lv_obj_align(mc_tone_shroud, LV_ALIGN_BOTTOM_LEFT, 0, 13);
    lv_obj_set_style_bg_color(mc_tone_shroud, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(mc_tone_shroud, 0, 0);
    lv_obj_set_style_radius(mc_tone_shroud, 0, 0);
    lv_obj_clear_flag(mc_tone_shroud, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(mc_tone_shroud, LV_SCROLLBAR_MODE_OFF);

    mc_tone_shroud_hump = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_shroud_hump, 184, 184);
    lv_obj_align(mc_tone_shroud_hump, LV_ALIGN_CENTER, 0, 100);
    lv_obj_set_style_bg_color(mc_tone_shroud_hump, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(mc_tone_shroud_hump, 0, 0);
    lv_obj_set_style_radius(mc_tone_shroud_hump, 92, 0);

    // Bottom container for labels
    mc_tone_bottom = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_bottom, 240, 120);
    lv_obj_align(mc_tone_bottom, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_opa(mc_tone_bottom, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_tone_bottom, 0, 0);

    // Center label (dB value)
    mc_tone_label_center = lv_label_create(mc_tone_bg);
    lv_obj_set_style_text_font(mc_tone_label_center, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(mc_tone_label_center, MC_TEXT_COLOR, 0);
    lv_obj_set_style_text_align(mc_tone_label_center, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(mc_tone_label_center, LV_ALIGN_CENTER, 0, 10);

    // Frequency label
    mc_tone_label_frequency = lv_label_create(mc_tone_bottom);
    lv_obj_set_style_text_font(mc_tone_label_frequency, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(mc_tone_label_frequency, MC_TEXT_COLOR, 0);
    lv_obj_set_style_text_align(mc_tone_label_frequency, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(mc_tone_label_frequency, LV_ALIGN_CENTER, 0, -25);

    // Bottom label (Tone Low / Tone High)
    mc_tone_label_bottom = lv_label_create(mc_tone_bottom);
    lv_obj_set_style_text_font(mc_tone_label_bottom, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(mc_tone_label_bottom, MC_TEXT_COLOR, 0);
    lv_obj_set_style_text_align(mc_tone_label_bottom, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(mc_tone_label_bottom, LV_ALIGN_BOTTOM_MID, 0, -30);

    lv_label_set_text(
        mc_tone_label_bottom,
        (band == TONE_LOW) ? "TONE LOW" : "TONE HIGH"
    );

    // Outer ring border
    mc_tone_outer_ring = lv_obj_create(parent);
    lv_obj_set_size(mc_tone_outer_ring, 240, 240);
    lv_obj_center(mc_tone_outer_ring);
    lv_obj_set_style_bg_opa(mc_tone_outer_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mc_tone_outer_ring, 8, 0);
    lv_obj_set_style_border_color(mc_tone_outer_ring, MC_BORDER_GREY, 0);
    lv_obj_set_style_radius(mc_tone_outer_ring, 120, 0);
}

// ================= PUBLIC API =================

void tone_page_create(lv_obj_t* parent, ToneBand band)
{
    s_band = band;
    use_mcintosh = (settings_get_theme() == THEME_MCINTOSH);

    if (use_mcintosh) {
        mcintosh_tone_create(parent, band);
    } else {
        basic_tone_create(parent, band);
    }

    tone_page_set_db_internal();
}

void tone_page_on_enter(ToneBand band)
{
    s_band = band;
    tone_page_set_db_internal();
}

void tone_page_delta(ToneBand band, int delta)
{
    helix_tone_delta(band, delta);
}

void tone_page_refresh(void)
{
    if (!helix_tone_enabled())
        return;

    tone_page_set_db_internal();
}
