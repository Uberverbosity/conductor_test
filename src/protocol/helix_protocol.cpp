#include "protocol/helix_protocol.h"
#include <Arduino.h>
#include "lvgl.h"
#include "pages/volume_page.h"
#include "pages/page_manager.h"
#include "pages/preset_page.h"

//#include "fonts/lv_font_montserrat_64.h"

// ================= CONFIG =================

#define FRAME_MAX        160
#define GAP_US           300
#define FRAME_DELAY_US   600

#define VOL_MIN 0x30     // DSP base volume code
#define DSP_SILENCE_MS  3000

// ================= SERIAL =================

static HardwareSerial* dsp = nullptr;

// ================= DSP LIVENESS =================

static uint32_t lastDspRxMs = 0;
static uint32_t lastRetryMs = 0;
static uint32_t hsStartMs = 0;
static bool warnedNoRx = false;
static uint32_t lastUserInteractionMs = 0;

// ================= HANDSHAKE STATE =================

enum HelixHS {
    HS_IDLE,
    HS_WAIT_ACK0,
    HS_WAIT_BLOB,
    HS_WAIT_SLOT_SNAP,
    HS_WAIT_FC,
    HS_WAIT_F2,
    HS_WAIT_SLOT2,
    HS_WAIT_TONE,
    HS_WAIT_FINAL,
    HS_WAIT_FD_CHALLENGE,
    HS_READY
};

static bool ownershipLatched = false;
static HelixHS hsState = HS_IDLE;
static bool ready = false;
static bool handshakeLocked = false;
static bool handshakeInProgress = false;
static uint32_t hs7AckMs = 0;
static bool ui_bound = false;
static bool dspOnline = false;
static bool rehandshakeFromFD = false;

// ================= MENU STATES =================
static bool toneMenuEnabled        = false;
static bool signalInputMenuEnabled = false;
static bool soundSetupMenuEnabled  = false;
static bool bluetoothMenuEnabled   = false;
static bool autoReturnEnabled = false;

// ================= TONE STATES =================
static bool     toneValid[2] = {false, false};
static int8_t   toneDb[2]     = {0, 0};
static uint16_t toneHz[2]    = {0, 0};

// ================= PRESET STATE =================
static uint8_t currentPreset = 0;

// ================= HANDSHAKE PACKETS =================

static const uint8_t HS0[] = {0x42,0x03,0xFC,0x01,0x2A,0x00,0x2A};
static const uint8_t HS1[] = {0x42,0x03,0xFC,0x01,0x2A,0x03,0x2D};
static const uint8_t HS2[] = {0x42,0x03,0xFC,0x01,0x2A,0x04,0x2E};
static const uint8_t HS3[] = {0x42,0x03,0xFC,0x01,0x2A,0x05,0x2F};
static const uint8_t HS4[] = {0x42,0x03,0xFC,0x01,0x2A,0x06,0x30};
static const uint8_t HS5[] = {0x42,0x03,0xFC,0x01,0x2A,0x07,0x31};
static const uint8_t HS6[] = {0x42,0x03,0xFC,0x01,0x2A,0x09,0x33};
static const uint8_t HS7[] = {0x42,0x05,0xFA,0x01,0x2B,0x01,0x01,0x01,0x2E};
static const uint8_t HS_FD_REPLY[] = {0x42, 0x06,0xF9, 0x01, 0x33, 0x00, 0x02, 0x01, 0x06, 0x3C};

// ================= RX CAPTURE =================

static uint8_t  capBuf[FRAME_MAX];
static size_t   capLen = 0;
static uint32_t lastByteUs = 0;

// ================= VOLUME MODEL =================

struct VolumeModel {
    uint8_t assign;
    uint8_t steps;
    float   range_dB;
    float   step_dB;
    uint8_t index;
    bool    valid;
    uint8_t color_r;
    uint8_t color_g;
    uint8_t color_b;
};

#define NUM_SLOTS 4
#define SLOT_STRIDE 10
#define SLOT_BASE0 22

static VolumeModel volume[NUM_SLOTS] = {};
static uint8_t activeSlot = 0;

static const uint8_t SLOT_BASE[NUM_SLOTS] = {
    SLOT_BASE0 + 0 * SLOT_STRIDE,  // VOL1
    SLOT_BASE0 + 1 * SLOT_STRIDE,  // VOL2
    SLOT_BASE0 + 2 * SLOT_STRIDE,  // VOL3
    SLOT_BASE0 + 3 * SLOT_STRIDE   // VOL4
};

#ifdef DEV_MODE
static void dev_seed_volume_models()
{
    struct SlotSeed {
        uint8_t assign;
        uint8_t steps;
        uint8_t index;
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    const SlotSeed seeds[NUM_SLOTS] = {
        { 0x00, 60, 30, 0x5C, 0xB8, 0xFF }, // Master
        { 0x01, 60, 24, 0xFF, 0x88, 0x55 }, // Subwoofer
        { 0x02, 60, 36, 0x93, 0xD7, 0xFF }, // Digital
        { 0x03, 60, 18, 0xA0, 0xFF, 0xA0 }  // AUX/HEC 1
    };

    for (uint8_t s = 0; s < NUM_SLOTS; s++) {
        const SlotSeed &seed = seeds[s];
        VolumeModel &vm = volume[s];

        vm.assign = seed.assign;
        vm.steps = seed.steps;
        vm.range_dB = 60.0f;
        vm.step_dB = vm.range_dB / (float)vm.steps;
        vm.index = seed.index;
        vm.valid = true;
        vm.color_r = seed.r;
        vm.color_g = seed.g;
        vm.color_b = seed.b;
    }
}

void helix_dev_init()
{
    ready = true;
    dspOnline = false;
    handshakeLocked = true;
    handshakeInProgress = false;
    hsState = HS_READY;
    ownershipLatched = true;
    warnedNoRx = false;
    lastDspRxMs = 0;
    lastRetryMs = 0;
    lastUserInteractionMs = 0;

    toneMenuEnabled = true;
    signalInputMenuEnabled = true;
    soundSetupMenuEnabled = false;
    bluetoothMenuEnabled = false;
    autoReturnEnabled = false;

    toneValid[TONE_LOW] = true;
    toneValid[TONE_HIGH] = true;
    toneDb[TONE_LOW] = 0;
    toneDb[TONE_HIGH] = 0;
    toneHz[TONE_LOW] = 80;
    toneHz[TONE_HIGH] = 8000;

    currentPreset = 0;
    activeSlot = 0;

    dev_seed_volume_models();

    Serial.println("[DEV] helix_dev_init: mock protocol state ready");
}
#endif

// ================= UTILS =================

static uint16_t u16le(const uint8_t *b)
{
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static void printHex(const char* tag, const uint8_t* buf, size_t len)
{
    Serial.printf("%s (%u): ", tag, (unsigned)len);
    for (size_t i = 0; i < len; i++)
        Serial.printf("%02X ", buf[i]);
    Serial.println();
}

void helix_note_user_interaction()
{
    lastUserInteractionMs = millis();
}

static void sendHS(const uint8_t* pkt, size_t len, const char* label)
{
    printHex("[TX HS]", pkt, len);
    dsp->write(pkt, len);
    Serial.printf("[HELIX] %s send\n", label);
}

static const char* slot_label_from_assign(uint8_t assign)
{
    switch (assign) {
        case 0x00: return "MASTER\nVOLUME";
        case 0x01: return "SUB\nLEVEL";
        case 0x02: return "DIGITAL\nLEVEL";
        case 0x03: return "AUX/HEC 1\nLEVEL";
        case 0x04: return "AUX/HEC 2\nLEVEL";
        case 0x05: return "REAR ATTENUATION\nLEVEL";
        default:   return "";
    }
}

static const char* slot_assign_name(uint8_t assign)
{
    switch (assign) {
        case 0x00: return "Master";
        case 0x01: return "Subwoofer";
        case 0x02: return "Digital";
        case 0x03: return "AUX/HEC 1";
        case 0x04: return "AUX/HEC 2";
        case 0x05: return "Rear Attenuation";
        default:   return "Unknown";
    }
}

static void update_tone_from_frame(const uint8_t* buf)
{
    toneDb[TONE_LOW]  = (int8_t)buf[7];
    toneDb[TONE_HIGH] = (int8_t)buf[11];
    toneValid[TONE_LOW]  = true;
    toneValid[TONE_HIGH] = true;
}

static void printConfigSummary()
{
    Serial.println("\n=== Menu Configuration ===");
    Serial.println("Main Menu: Volume Control");
    Serial.print("Auto Return After 5 Seconds: ");
    Serial.println(autoReturnEnabled ? "Enabled" : "Disabled");
    Serial.print("Tone: ");
    Serial.println(toneMenuEnabled ? "Enabled" : "Disabled");
    Serial.print("Signal Input: ");
    Serial.println(signalInputMenuEnabled ? "Enabled" : "Disabled");
    Serial.print("Sound Setup: ");
    Serial.println(soundSetupMenuEnabled ? "Enabled" : "Disabled");
    Serial.print("Bluetooth: ");
    Serial.println(bluetoothMenuEnabled ? "Enabled" : "Disabled");

    Serial.println("=== Volume Control Configuration ===");
    for (int s = 0; s < NUM_SLOTS; s++) {
        VolumeModel &vm = volume[s];
        Serial.printf("[VOL%d] ", s + 1);
        if (!vm.valid) {
            Serial.println("Disabled");
            continue;
        }
        Serial.printf(
            "Assigned=%s, Color=#%02X%02X%02X, Steps=%u, Range=%u dB, Step Size=%.2f dB, Startup Limiter=Disabled\n",
            slot_assign_name(vm.assign),
            vm.color_r, vm.color_g, vm.color_b,
            vm.steps,
            (uint16_t)vm.range_dB,
            vm.step_dB
        );
    }

    Serial.println("=== Tone Control ===");
    if (toneValid[TONE_LOW]) {
        Serial.printf("[TONE LOW] %u Hz, %+d dB\n", toneHz[TONE_LOW], toneDb[TONE_LOW]);
    }
    if (toneValid[TONE_HIGH]) {
        Serial.printf("[TONE HIGH] %u Hz, %+d dB\n", toneHz[TONE_HIGH], toneDb[TONE_HIGH]);
    }
    Serial.println("=========================");
    Serial.println();
}

static void printToneStates()
{
    if (toneMenuEnabled) {
        Serial.println("\n=== TONE CONFIG ===");

        Serial.print("Low Corner: ");
        Serial.print(toneHz[TONE_LOW]);
        Serial.println(" Hz");

        Serial.print("Low Gain: ");
        if (toneValid[TONE_LOW]) {
            Serial.printf("%+d dB\n", toneDb[TONE_LOW]);
        } else {
            Serial.println("Unknown");
        }

        Serial.print("High Corner: ");
        Serial.print(toneHz[TONE_HIGH]);
        Serial.println(" Hz");

        Serial.print("High Gain: ");
        if (toneValid[TONE_HIGH]) {
            Serial.printf("%+d dB\n", toneDb[TONE_HIGH]);
        } else {
            Serial.println("Unknown");
        }

        Serial.println("===================\n");
    }
}

// ================= HANDSHAKE RESET =================

static void resetHandshake()
{
    Serial.println("[HELIX] handshake reset");
    
    dspOnline = false;
    warnedNoRx = false;
    lastDspRxMs = 0;
    hsStartMs = millis();

    ready = false;
    handshakeLocked = false;
    handshakeInProgress = true;
    ownershipLatched = false;
    for (int s = 0; s < NUM_SLOTS; s++) {
        volume[s].valid = false;
    }
    capLen = 0;
    hsState = HS_WAIT_ACK0;

    // Reset Tone States
    toneValid[TONE_LOW]  = false;
    toneValid[TONE_HIGH] = false;
    toneHz[TONE_LOW]     = 0;
    toneHz[TONE_HIGH]    = 0;

    sendHS(HS0, sizeof(HS0), "HS0");
}

// ================= DECODERS =================

static void decode_volume_blob(const uint8_t *buf)
{
    if (hsState < HS_WAIT_BLOB)
        return;

    for (int s = 0; s < NUM_SLOTS; s++) {

        VolumeModel &vm = volume[s];
        const int o = SLOT_BASE[s];

        uint8_t assign = buf[o];        // slot assignment
        uint8_t steps  = buf[o + 4];    // number of steps

        vm.assign = assign;

        // ---- VALIDITY GATE ----
        if (assign == 0xFF || steps == 0) {
            vm.valid = false;
            Serial.printf("[VOL%d] disabled\n", s + 1);
            continue;
        }

        float range_dB = u16le(&buf[o + 6]) / 10.0f;

        vm.steps    = steps;
        vm.range_dB = range_dB;
        vm.step_dB  = range_dB / steps;
        vm.valid    = true;

        // ---- SLOT COLOR ----
        uint8_t r = buf[o + 1];
        uint8_t g = buf[o + 2];
        uint8_t b = buf[o + 3];

        vm.color_r = r;
        vm.color_g = g;
        vm.color_b = b;

        volume_page_set_color((DialSlot)s, r, g, b);

        Serial.printf(
            "[VOL%d] assign=%02X steps=%u range=%.1f step=%.2f\n",
            s + 1,
            assign,
            vm.steps,
            vm.range_dB,
            vm.step_dB
        );

    }

    // ---- Initialize active slot once config is known ----
    if (activeSlot >= NUM_SLOTS || !volume[activeSlot].valid) {
        // Find first valid slot
        for (int s = 0; s < NUM_SLOTS; s++) {
            if (volume[s].valid) {
                helix_set_active_slot(s);
                break;
            }
        }
    }

    page_manager_refresh();

}



static void decode_volume_snapshot(const uint8_t *buf)
{
    const int SNAP_BASE = 6;

    for (int s = 0; s < NUM_SLOTS; s++) {

        VolumeModel &vm = volume[s];
        if (!vm.valid)
            continue;

        uint8_t idx = buf[SNAP_BASE + s];
        if (idx > vm.steps)
            idx = vm.steps;

        vm.index = idx;

        Serial.printf(
            "[SNAP] slot=%d idx=%d %s\n",
            s,
            vm.index,
            (s == activeSlot) ? "<ACTIVE>" : ""
        );

        if (s == activeSlot) {
            int ui = map(vm.index, 0, vm.steps, 0, 100);
            volume_page_set_absolute(ui);
        }
    }
}


static int slot_from_fa(uint8_t slot_id)
{
    switch (slot_id) {
        case 0x90: return 0;  // VOL1
        case 0x91: return 1;  // VOL2
        case 0x92: return 2;  // VOL3
        case 0x93: return 3;  // VOL4
        default:   return -1;
    }
}

static inline bool is_valid_slot(uint8_t slot)
{
    return slot < NUM_SLOTS && volume[slot].valid;
}

void helix_set_active_slot(uint8_t slot)
{
    if (!is_valid_slot(slot))
        return;

    VolumeModel &vm = volume[slot];

    // Prevent re-entrant spam
    if (activeSlot == slot)
        return;

    activeSlot = slot;

    Serial.printf("[HELIX] active slot = %u\n", slot);

    // ---- ALWAYS update slot context ----
    volume_page_set_slot((DialSlot)slot);
    volume_page_set_label(
        slot_label_from_assign(vm.assign)
    );

    int ui = map(vm.index, 0, vm.steps, 0, 100);
    volume_page_set_absolute(ui);
}

void helix_cycle_slot()
{
    // Find current slot index in valid slots list
    uint8_t validSlots[NUM_SLOTS];
    uint8_t numValid = 0;
    uint8_t currentIdx = 0xFF;

    for (uint8_t s = 0; s < NUM_SLOTS; s++) {
        if (volume[s].valid) {
            validSlots[numValid++] = s;
            if (s == activeSlot) {
                currentIdx = numValid - 1;
            }
        }
    }

    if (numValid < 2)
        return;

    // Move to next valid slot
    uint8_t nextIdx = (currentIdx + 1) % numValid;
    helix_set_active_slot(validSlots[nextIdx]);
}

DialSlot helix_get_active_slot()
{
    return DialSlot(activeSlot);
}

const char* helix_get_slot_label(DialSlot slot)
{
    if (!is_valid_slot(slot))
        return "";

    return slot_label_from_assign(volume[slot].assign);
}

int helix_get_slot_ui_value(DialSlot slot)
{
    if (!is_valid_slot(slot))
        return 0;

    VolumeModel &vm = volume[slot];
    return map(vm.index, 0, vm.steps, 0, 100);
}

bool helix_slot_valid(uint8_t slot)
{
    return is_valid_slot(slot);
}

void helix_force_resync()
{
#ifdef DEV_MODE
    Serial.println("[DEV] helix_force_resync ignored");
    return;
#endif
    Serial.println("[HELIX] manual resync requested");

    handshakeLocked = false;
    handshakeInProgress = false;
    ready = false;

    resetHandshake();
}

void helix_ui_bind_complete()
{
    ui_bound = true;

#ifdef DEV_MODE
    for (uint8_t s = 0; s < NUM_SLOTS; s++) {
        if (!volume[s].valid)
            continue;
        volume_page_set_color(
            (DialSlot)s,
            volume[s].color_r,
            volume[s].color_g,
            volume[s].color_b
        );
    }
    page_manager_refresh();
#endif

    if (rehandshakeFromFD) {
        Serial.println("[UI] Reinitializing page manager after re-handshake");

        page_manager_init(lv_scr_act());

        rehandshakeFromFD = false;
    }
}

uint8_t helix_get_current_preset()
{
    return currentPreset;
}

// ================= FRAME PROCESSOR =================

static void processFrame()
{
    // Minimum frame size check (frames should be at least 3 bytes: header + type)
    if (capLen < 3) {
        Serial.printf("[HELIX] Dropping invalid frame (len=%u)\n", capLen);
        return;
    }

    uint8_t type = capBuf[2];

    // Any non-FD frame implies DSP is alive
    if (type != 0xFD) {
        dspOnline = true;
        lastDspRxMs = millis();
    }

    // Live color preview (no resync)
    if (type == 0xFA && capBuf[4] == 0x33 && capLen >= 9) {

        uint8_t slot_id = capBuf[5];
        uint8_t r = capBuf[6];
        uint8_t g = capBuf[7];
        uint8_t b = capBuf[8];

        DialSlot slot = (DialSlot)slot_from_fa(slot_id);
        if (slot >= 0 && slot < NUM_SLOTS) {
            Serial.printf(
                "[UI] live color slot=%d #%02X%02X%02X\n",
                slot, r, g, b
            );
            volume_page_set_color(slot, r, g, b);
        }
        return;
    }

    if (type == 0xFD && capBuf[4] == 0x33) {
        uint8_t reason = capBuf[5];

        Serial.printf("[HELIX] FD obj=33 reason=%02X\n", reason);

        if (reason == 0x02) {
            Serial.println("[HELIX] FD config change → full re-handshake");

            rehandshakeFromFD = true;

            resetHandshake();
            return;
        }

        // reason == 0x00 → ownership challenge / keepalive
    }


    printHex("[RX]", capBuf, capLen);

    // ---- Always decode config/state ----
    if (type == 0xAF) {
        Serial.println("[DBG] AF blob received");
        decode_volume_blob(capBuf);

        autoReturnEnabled = capBuf[8];
        toneMenuEnabled        = capBuf[21];
        signalInputMenuEnabled = capBuf[19];
        soundSetupMenuEnabled  = capBuf[18];
        bluetoothMenuEnabled   = capBuf[20];

        toneHz[TONE_LOW]  = u16le(&capBuf[72]);
        toneHz[TONE_HIGH] = u16le(&capBuf[82]);
    }

    if (type == 0xF9 && capBuf[4] == 0x2A && capBuf[5] == 0x04) {
        decode_volume_snapshot(capBuf);
    }

    // ---- RUNTIME TONE UPDATE ----
    if (type == 0xF5 && capBuf[5] == 0x09 && !handshakeInProgress) {
        update_tone_from_frame(capBuf);

        Serial.printf(
            "[TONE RX] Low=%u Hz, %+d dB\n",
            toneHz[TONE_LOW],
            toneDb[TONE_LOW]
        );
        Serial.printf(
            "[TONE RX] High=%u Hz, %+d dB\n",
            toneHz[TONE_HIGH],
            toneDb[TONE_HIGH]
        );

        page_manager_refresh();
    }

    // ---- ACK OF PRESET CHANGE ----
    if (type == 0xFB &&
        capBuf[4] == 0x2B &&
        capBuf[5] == 0x06)
    {
        uint8_t idx    = capBuf[6];
        uint8_t status = capBuf[7];   // 01 = OK, 00 = No Config

        preset_page_on_ack(idx, status);
    }
   
    // ---- GET CURRENT PRESET ----
    if (type == 0xF2 &&
        capBuf[4] == 0x2A &&
        capBuf[5] == 0x06)
    {
        currentPreset = capBuf[6];
        // Refresh preset page if it's currently active to sync UI
        page_manager_refresh();
    }

    // ---- Handshake FSM ----
    if (handshakeLocked) {
        return;
    }

    switch (hsState) {

    case HS_WAIT_ACK0:
        if (type == 0xFB && capBuf[4] == 0x2A) {
            sendHS(HS1, sizeof(HS1), "HS1");
            hsState = HS_WAIT_BLOB;
        }
        break;

    case HS_WAIT_BLOB:
        if (type == 0xAF) {
            sendHS(HS2, sizeof(HS2), "HS2");
            hsState = HS_WAIT_SLOT_SNAP;
        }
        break;

    case HS_WAIT_SLOT_SNAP:
        if (type == 0xF9 && capBuf[5] == 0x04) {
            sendHS(HS3, sizeof(HS3), "HS3");
            hsState = HS_WAIT_FC;
        }
        break;

    case HS_WAIT_FC:
        if (type == 0xFC) {
            sendHS(HS4, sizeof(HS4), "HS4");
            hsState = HS_WAIT_F2;
        }
        break;

    case HS_WAIT_F2:
        if (type == 0xF2) {
            sendHS(HS5, sizeof(HS5), "HS5");
            hsState = HS_WAIT_SLOT2;
        }
        break;

    case HS_WAIT_SLOT2:
        if (type == 0xF9 && capBuf[5] == 0x07) {
            delayMicroseconds(250);
            sendHS(HS6, sizeof(HS6), "HS6");
            hsState = HS_WAIT_TONE;
        }
        break;

    case HS_WAIT_TONE:
        if (type == 0xF5 && capBuf[5] == 0x09) {
            update_tone_from_frame(capBuf);

            Serial.printf(
                "[TONE RX] Low=%u Hz, %+d dB\n",
                toneHz[TONE_LOW],
                toneDb[TONE_LOW]
            );
            Serial.printf(
                "[TONE RX] High=%u Hz, %+d dB\n",
                toneHz[TONE_HIGH],
                toneDb[TONE_HIGH]
            );

            sendHS(HS7, sizeof(HS7), "HS7");
            hsState = HS_WAIT_FINAL;
        }
        break;

    case HS_WAIT_FINAL:
        if (type == 0xFB && capBuf[4] == 0x2B) {

            ready = true;
            handshakeInProgress = false;

            // Force MASTER slot selection once config is known
            helix_set_active_slot(0);

            hsState = HS_WAIT_FD_CHALLENGE;
            hs7AckMs = millis();  // Initialize timestamp for FD challenge timeout
            printConfigSummary();

            Serial.println("[HELIX] HS7 ACK, READY asserted");
        }
        break;

    case HS_WAIT_FD_CHALLENGE:

        if (type == 0xFD && capBuf[4] == 0x33) {
            dsp->write(HS_FD_REPLY, sizeof(HS_FD_REPLY));
            dsp->flush();

            Serial.println("[HELIX] FD challenge answered");

            ownershipLatched    = true;
            handshakeLocked     = true;
            handshakeInProgress = false;

            hsState = HS_READY;
            Serial.println("[HELIX] READY (ownership latched)");
            break;
        }

        // Timeout ONLY allowed on cold boot
        if (!ownershipLatched &&
            millis() - hs7AckMs > 100)
        {
            ownershipLatched    = true;
            handshakeLocked     = true;
            handshakeInProgress = false;

            hsState = HS_READY;
            Serial.println("[HELIX] READY (no FD challenge)");
        }

        break;

    }
    
}

// ================= PUBLIC API =================

#ifndef DEV_MODE
void helix_dev_init()
{
    // No-op in production builds
}
#endif

void helix_begin(HardwareSerial& dspSerial)
{
#ifdef DEV_MODE
    (void)dspSerial;
    helix_dev_init();
    return;
#endif
    dsp = &dspSerial;

    hsState = HS_IDLE;
    handshakeLocked = false;
    handshakeInProgress = false;
    ready = false;

    static bool hs_started = false;

    if (!hs_started) {
        hs_started = true;
        Serial.println("[HELIX] waiting for DSP boot");
        delay(800);                 // one-time, blocking
        Serial.println("[HELIX] starting handshake");
        resetHandshake();
    }
}

bool helix_ready()
{
    return ready;
}

// ================== TONE CAPABILITY ================
bool helix_tone_enabled()      { return toneMenuEnabled; }
bool helix_tone_low_valid()    { return toneValid[TONE_LOW]; }
bool helix_tone_high_valid()   { return toneValid[TONE_HIGH]; }

// ================== SIGNAL INPUT CAPABILITY ================
bool helix_signal_input_enabled() { return signalInputMenuEnabled; }
int8_t helix_tone_low_db()     { return toneDb[TONE_LOW]; }
int8_t helix_tone_high_db()    { return toneDb[TONE_HIGH]; }
uint16_t helix_tone_low_hz()   { return toneHz[TONE_LOW]; }
uint16_t helix_tone_high_hz()  { return toneHz[TONE_HIGH]; }

void helix_tone_set(ToneBand band, int8_t db)
{
#ifdef DEV_MODE
    if (!ready)
        return;

    toneDb[band] = db;
    toneValid[band] = true;
    page_manager_refresh();
    return;
#endif
    if (!ready)
        return;

    uint8_t band_id = (band == TONE_LOW) ? 0x00 : 0x01;
    uint16_t freq   = toneHz[band];

    const uint8_t wake[] = { 0x42, 0x08 };
    dsp->write(wake, sizeof(wake));
    dsp->flush();
    delayMicroseconds(FRAME_DELAY_US);

    uint8_t pkt[] = {
        0xF7,
        0x01,
        0x2B,
        0x09,
        band_id,
        (uint8_t)db,
        (uint8_t)(freq >> 8),
        (uint8_t)(freq & 0xFF),
        0x01,
        0x00   // checksum
    };

    // OEM fixed-base checksum
    uint8_t base = (band == TONE_LOW) ? 0x62 : 0x0D;
    pkt[9] = base + pkt[5];

    printHex("[TX TONE]", pkt, sizeof(pkt));
    dsp->write(pkt, sizeof(pkt));
    dsp->flush();
}

void helix_tone_delta(ToneBand band, int delta)
{
    if (!ready)
        return;

#ifdef DEV_MODE
    int8_t current = toneDb[band];
    int8_t target = current + delta;

    if (target < -12) target = -12;
    if (target >  12) target =  12;
    if (target == current)
        return;

    toneDb[band] = target;
    toneValid[band] = true;
    page_manager_refresh();
#else
    int8_t current = toneDb[band];
    int8_t target = current + delta;

    if (target < -12) target = -12;
    if (target >  12) target =  12;
    if (target == current)
        return;

    helix_tone_set(band, target);
    toneDb[band] = target;

    page_manager_refresh();
#endif
}

// ================= PRESET SELECT ==============
bool helix_select_preset(uint8_t idx)
{
#ifdef DEV_MODE
    if (!ready || idx > 9)
        return false;

    currentPreset = idx;
    preset_page_on_ack(idx, 0x01);
    return true;
#endif
    if (!ready || !dsp || idx > 9)
        return false;

    uint8_t pkt[9] = {
        0x42, 0x05,
        0xFA, 0x01,
        0x2B, 0x06,
        idx,
        0x01,
        (uint8_t)(0x32 + idx)   // ← checksum, final byte
    };

    printHex("[TX PRESET]", pkt, sizeof(pkt));
    dsp->write(pkt, sizeof(pkt));
    dsp->flush();

    return true;
}

// ================= LOOP =================

void helix_loop()
{
    if (!dsp) return;
    uint32_t nowUs = micros();
    
    while (dsp->available()) {
        uint8_t b = dsp->read();

        if (capLen < FRAME_MAX)
            capBuf[capLen++] = b;

        lastByteUs = nowUs;
    }

    if (capLen && (nowUs - lastByteUs) > GAP_US) {
        processFrame();
        capLen = 0;
    }

    if (!ready && handshakeInProgress && !warnedNoRx) {
        if (millis() - hsStartMs > 1000 &&
            (lastDspRxMs == 0 || millis() - lastDspRxMs > 1000))
        {
            Serial.println(
                "[WARN] No DSP RX after HS0.  Is the ESP connected?  Is OEM conductor still attached?"
            );
            warnedNoRx = true;
        }
    }

    if (handshakeInProgress &&
        millis() - hsStartMs > 1500 &&
        !dspOnline)
    {
        Serial.println("[HELIX] DSP appears offline, backing off handshake");

        handshakeInProgress = false;
        handshakeLocked = true;
        hsState = HS_IDLE;
    }

    if (!ready &&
        !dspOnline &&
        millis() - lastRetryMs > 3000)
    {
        Serial.println("[HELIX] retrying DSP handshake");
        resetHandshake();
        lastRetryMs = millis();
    }

    if (ready &&
        autoReturnEnabled &&
        lastUserInteractionMs != 0 &&
        millis() - lastUserInteractionMs > 5000)
    {
        // Semantic return to Master Volume ONLY
        helix_set_active_slot(0);
        page_manager_set_to_master_volume();

        lastUserInteractionMs = millis(); // prevent repeat firing
    }
}

// ================= VOLUME DELTA =================

void helix_volume_delta(int8_t clicks)
{
    Serial.println("[DBG] helix_volume_delta CALLED");
    if (!ready)
        return;

    VolumeModel &vm = volume[activeSlot];

    if (!vm.valid)
        return;

    int next = (int)vm.index + clicks;

    if (next < 0)           next = 0;
    if (next > vm.steps)    next = vm.steps;
    if (next == vm.index)   return;

    vm.index = (uint8_t)next;

#ifdef DEV_MODE
    int ui = map(vm.index, 0, vm.steps, 0, 100);
    volume_page_set_absolute(ui);
#else
    uint8_t volCode = VOL_MIN + vm.index + activeSlot;

    const uint8_t wake[] = { 0x42, 0x06 };
    dsp->write(wake, sizeof(wake));
    dsp->flush();
    delayMicroseconds(FRAME_DELAY_US);

    uint8_t body[] = {
        0xF9, 0x01, 0x2B, 0x04,
        activeSlot,     // ← slot selector (0–3)
        vm.index,
        0x01,
        volCode
    };

    printHex("[TX VOL]", body, sizeof(body));
    dsp->write(body, sizeof(body));
    dsp->flush();

    int ui = map(vm.index, 0, vm.steps, 0, 100);
    volume_page_set_absolute(ui);
#endif
}

