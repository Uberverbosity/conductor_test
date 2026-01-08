#include "tone_page.h"
#include "protocol/helix_protocol.h"
#include <lvgl.h>
#include <cstdio>

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
    lv_obj_t* label_bottom;
};

static TonePageCtx ctx;
static ToneBand s_band;

// ================= ARC COLORS =====================
static lv_color_t tone_arc_color(ToneBand band)
{
    return (band == TONE_LOW)
        ? lv_color_hex(0x7B3FE4)   // Purple
        : lv_color_hex(0xFF8C1A);  // Orange
}

// ================= INTERNAL HELPERS =================

static void tone_page_set_db_internal(void)
{
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
        lv_label_set_text(ctx.label_center, "--");
        return;
    }

    // ----- CENTER LABEL -----
    char buf[8];
    snprintf(buf, sizeof(buf), "%+d", db);
    lv_label_set_text(ctx.label_center, buf);

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

// ================= PUBLIC API =================

void tone_page_create(lv_obj_t* parent, ToneBand band)
{
    s_band = band;

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
        lv_color_hex(0xFFFFFF),
        0
    );

    // ----- BOTTOM LABEL -----
    ctx.label_bottom = lv_label_create(parent);
    lv_obj_set_style_text_font(
        ctx.label_bottom,
        &lv_font_montserrat_20,
        0
    );
    lv_obj_set_style_text_color(
        ctx.label_bottom,
        lv_color_hex(0xFFFFFF),
        0
    );
    lv_obj_align(ctx.label_bottom, LV_ALIGN_BOTTOM_MID, 0, -35);

    lv_label_set_text(
        ctx.label_bottom,
        (band == TONE_LOW) ? "Tone Low" : "Tone High"
    );

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
