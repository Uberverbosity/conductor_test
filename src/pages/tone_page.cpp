#include "tone_page.h"
#include "protocol/helix_protocol.h"
#include <cstdio>

// ================= INTERNAL UI OBJECTS =================

struct TonePageCtx {
    lv_obj_t* label_center;
    lv_obj_t* label_bottom;
};

static TonePageCtx ctx;
static ToneBand s_band;

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

    char buf[8];
    snprintf(buf, sizeof(buf), "%+d", db);
    lv_label_set_text(ctx.label_center, buf);
}

// ================= PUBLIC API =================

void tone_page_create(lv_obj_t* parent, ToneBand band)
{
    s_band = band;

    // ----- CENTER LABEL (MATCH VOLUME PAGE) -----
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

    // ----- BOTTOM LABEL (MATCH dial_function EXACTLY) -----
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
    Serial.printf("[TONE] band=%d delta=%d\n", band, delta);
    // DSP command will go here later
}

void tone_page_refresh(void)
{
    if (!helix_tone_enabled())
        return;

    tone_page_set_db_internal();
}
