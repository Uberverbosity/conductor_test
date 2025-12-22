#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

#include "pages/master_dial.h"
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

// ================== ENCODER (POLLING) ==================
static uint8_t enc_prev = 0;
static int enc_accum = 0;

// Quadrature decode table
static const int8_t quad_table[4][4] = {
    {  0, -1, +1,  0 },
    { +1,  0,  0, -1 },
    { -1,  0,  0, +1 },
    {  0, +1, -1,  0 }
};

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

static bool btn_prev = true;   // INPUT_PULLUP = idle HIGH

static inline void poll_encoder_button()
{
    bool now = digitalRead(PIN_ENC_BTN);

    // Falling edge = button press
    if (btn_prev && !now) {
        Serial.println("[BTN] Encoder pressed");
        helix_cycle_slot();   // <-- slot cycling hook
    }

    btn_prev = now;
}

// ================== SETUP ==================
void setup()
{
    Serial.begin(115200);
    delay(200);

    // GPIO
    pinMode(PIN_BL, OUTPUT);
    digitalWrite(PIN_BL, LOW);

    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_ENC_BTN, INPUT_PULLUP);

    // LVGL
    lv_init();

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    lv_display_t* disp = lv_display_create(240, 240);

    static uint16_t buf1[240 * 40];
    static uint16_t buf2[240 * 40];

    lv_display_set_buffers(
        disp,
        buf1, buf2,
        sizeof(buf1),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    lv_display_set_flush_cb(disp, my_flush_cb);

    master_dial_create(lv_scr_act());

    // DSP UART
    Serial1.begin(
        230400,
        SERIAL_8N1,
        DSP_RX_PIN,
        DSP_TX_PIN
    );

    delay(300);
    helix_begin(Serial1);

    // Initialize encoder state
    enc_prev = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

    Serial.println("[BOOT] Setup complete");
}

// ================== LOOP ==================
void loop()
{
    // --- Protocol ---
    helix_loop();

    // --- LVGL ---
    static uint32_t last = 0;
    uint32_t now = millis();
    lv_tick_inc(now - last);
    last = now;
    lv_timer_handler();

    // --- Encoder Poll (1–2 kHz effective) ---
    poll_encoder();
    poll_encoder_button();

    // Drain encoder
    if (enc_accum != 0) {
        int delta = enc_accum;
        enc_accum = 0;

        Serial.printf("[ENC] delta=%d\n", delta);
        helix_volume_delta(delta);
    }

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'h') {
            helix_force_resync();
        }
        if (c == 'r') {
            ESP.restart();
        }
    }

    delay(2);
}
