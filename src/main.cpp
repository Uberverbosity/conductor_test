#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

#include "pages/volume_page.h"
#include "pages/page_manager.h"
#include "protocol/helix_protocol.h"

// ================== PINS ==================
#define PIN_BL      8
#define PIN_ENC_A   6
#define PIN_ENC_B   7
#define PIN_ENC_BTN 9

#define DSP_RX_PIN  20
#define DSP_TX_PIN  21

// ================== DISPLAY ==================
TFT_eSPI tft = TFT_eSPI();

void my_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)color_p, w * h, true);
    tft.endWrite();

    lv_display_flush_ready(disp);
}

// ================== ENCODER ==================
static uint8_t enc_prev = 0;
static int enc_accum = 0;

static const int8_t quad_table[4][4] = {
    {  0, -1, +1,  0 },
    { +1,  0,  0, -1 },
    { -1,  0,  0, +1 },
    {  0, +1, -1,  0 }
};

static bool btn_prev = true;

static bool btn_down = false;
static uint32_t btn_down_ms = 0;
static const uint32_t LONG_PRESS_MS = 400;

static bool long_press_fired = false;

static inline void poll_encoder_button()
{
    bool now = digitalRead(PIN_ENC_BTN);

    // Button down
    if (!now && btn_prev) {
        btn_down = true;
        btn_down_ms = millis();
        long_press_fired = false;
    }

    // Button held
    if (!now && btn_down && !long_press_fired) {
        if (millis() - btn_down_ms >= LONG_PRESS_MS) {
            long_press_fired = true;
            page_manager_encoder_long_press();
        }
    }

    // Button released
    if (now && !btn_prev && btn_down) {
        btn_down = false;

        if (!long_press_fired) {
            // short press
            page_manager_encoder_button();
        }
    }

    btn_prev = now;
}

static inline void poll_encoder()
{
    uint8_t a = digitalRead(PIN_ENC_A);
    uint8_t b = digitalRead(PIN_ENC_B);
    uint8_t cur = (a << 1) | b;

    int8_t delta = quad_table[enc_prev][cur];
    enc_prev = cur;

    if (delta != 0) {
        enc_accum += delta;
    }
}

static bool last_helix_ready = false;

// ================== SETUP ==================
void setup()
{
    Serial.begin(115200);
    delay(300);                     // <-- REQUIRED on ESP32
    Serial.println("[BOOT] setup enter");

    pinMode(PIN_BL, OUTPUT);
    digitalWrite(PIN_BL, LOW);

    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_ENC_BTN, INPUT_PULLUP);

    // ---- LVGL FIRST ----
    lv_init();

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    lv_display_t* disp = lv_display_create(240, 240);
    lv_display_set_default(disp);
    lv_display_set_flush_cb(disp, my_flush_cb);

    static uint16_t buf1[240 * 40];
    static uint16_t buf2[240 * 40];

    lv_display_set_buffers(
        disp,
        buf1,
        buf2,
        sizeof(buf1),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );
    
    // ---- UART SECOND ----
    Serial1.begin(
        230400,
        SERIAL_8N1,
        DSP_RX_PIN,
        DSP_TX_PIN
    );

    delay(300);
    Serial.println("[BOOT] UART up");

    helix_begin(Serial1);
    Serial.println("[BOOT] helix_begin returned");

    enc_prev = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

    Serial.println("[BOOT] setup complete");
}

// ================== LOOP ==================
void loop()
{
    static bool ui_initialized  = false;

    // ---- Protocol RX ----
    helix_loop();

    bool now_ready = helix_ready();
    if (now_ready && !last_helix_ready) {
        Serial.println("[HELIX] helix_ready transitioned to TRUE");
        // Call helix_ui_bind_complete() on ready transition to handle re-handshake UI reinit
        // (only if UI was already initialized, to avoid double init on first boot)
        if (ui_initialized) {
            helix_ui_bind_complete();
        }
    }
    last_helix_ready = now_ready;

    // ---- UI init once DSP is READY ----
    if (helix_ready() && !ui_initialized) {
        Serial.println("[UI] Initializing page manager");
        page_manager_init(lv_scr_act());
        helix_ui_bind_complete();  // Must be called after page_manager_init()
        ui_initialized = true;
    }

    // ---- LVGL tick ----
    static uint32_t last = 0;
    uint32_t now = millis();
    lv_tick_inc(now - last);
    last = now;
    lv_timer_handler();

    // ---- Encoder ----
    poll_encoder();
    poll_encoder_button();

    // Rotary delta (edge-based, stabilized)
    if (ui_initialized) {
        if (enc_accum >= 2) {
            enc_accum -= 2;
            page_manager_encoder_delta(+1);
        }
        else if (enc_accum <= -2) {
            enc_accum += 2;
            page_manager_encoder_delta(-1);
        }
    }
    // Debug console
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'h') helix_force_resync();
        if (c == 'r') ESP.restart();
    }

    delay(2);
}
