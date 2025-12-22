#include "helix_protocol.h"
#include <Arduino.h>
#include "pages/master_dial.h"

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

static HelixHS hsState = HS_IDLE;
static bool ready = false;
static bool handshakeLocked = false;
static bool handshakeInProgress = false;
static uint32_t hs7AckMs = 0;
uint8_t slot_id = 0;
static bool slotInitialized = false;


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
};

#define NUM_SLOTS 4
#define SLOT_STRIDE 10
#define SLOT_BASE0 22

static VolumeModel volume[NUM_SLOTS] = {};
static uint8_t activeSlot = 0;

static uint8_t validSlotIdx[NUM_SLOTS];
static uint8_t numValidSlots = 0;

static const uint8_t SLOT_BASE[NUM_SLOTS] = {
    SLOT_BASE0 + 0 * SLOT_STRIDE,  // VOL1
    SLOT_BASE0 + 1 * SLOT_STRIDE,  // VOL2
    SLOT_BASE0 + 2 * SLOT_STRIDE,  // VOL3
    SLOT_BASE0 + 3 * SLOT_STRIDE   // VOL4
};

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
        case 0x03: return "REAR\nLEVEL";
        default:   return "";
    }
}

// ================= HANDSHAKE RESET =================

static void resetHandshake()
{
    Serial.println("[HELIX] handshake reset");

    ready = false;
    handshakeLocked = false;
    handshakeInProgress = true;
    for (int s = 0; s < NUM_SLOTS; s++) {
        volume[s].valid = false;
    }
    capLen = 0;
    hsState = HS_WAIT_ACK0;

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

        master_dial_set_color((DialSlot)s, r, g, b);

        Serial.printf(
            "[VOL%d] assign=%02X steps=%u range=%.1f step=%.2f\n",
            s + 1,
            assign,
            vm.steps,
            vm.range_dB,
            vm.step_dB
        );
    }

    // ---- REBUILD VALID SLOT LIST ----
    numValidSlots = 0;
    for (int s = 0; s < NUM_SLOTS; s++) {
        if (volume[s].valid) {
            validSlotIdx[numValidSlots++] = s;
        }
    }

    // ---- Initialize active slot once config is known ----
    if (!slotInitialized && numValidSlots > 0) {
        helix_set_active_slot(validSlotIdx[0]);
        slotInitialized = true;
    }

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
            master_dial_set_absolute(ui);
        }
    }
}


static int slot_from_fa(uint8_t slot_id)
{
    switch (slot_id) {
        case 0x90: return DIAL_SLOT_VOL1;
        // future:
        // case 0x91: return DIAL_SLOT_VOL2;
        // case 0x92: return DIAL_SLOT_VOL3;
        // case 0x93: return DIAL_SLOT_VOL4;
        default:   return -1;
    }
}

void helix_set_active_slot(uint8_t slot)
{
    if (slot >= NUM_SLOTS)
        return;

    VolumeModel &vm = volume[slot];

    if (!vm.valid)
        return;

    activeSlot = slot;

    // ---- Update UI context (slot identity) ----
    master_dial_set_slot((DialSlot)slot);

    master_dial_set_label(
        slot_label_from_assign(vm.assign)
    );

    // ---- Update UI value (authoritative snapshot) ----
    int ui = map(vm.index, 0, vm.steps, 0, 100);
    master_dial_set_absolute(ui);

    Serial.printf("[HELIX] active slot = %u\n", slot);
}

void helix_cycle_slot()
{
    if (numValidSlots < 2)
        return;

    static uint8_t idx = 0;

    idx = (idx + 1) % numValidSlots;
    helix_set_active_slot(validSlotIdx[idx]);
}

uint8_t helix_get_active_slot()
{
    return activeSlot;
}


void helix_force_resync()
{
    Serial.println("[HELIX] manual resync requested");

    handshakeLocked = false;
    handshakeInProgress = false;
    ready = false;

    resetHandshake();
}

// ================= FRAME PROCESSOR =================

static void processFrame()
{
    uint8_t type = capBuf[2];

    // Live color preview (no resync)
    if (type == 0xFA && capBuf[4] == 0x33 && capLen >= 9) {

        uint8_t slot_id = capBuf[5];   // 0x90 = VOL1
        uint8_t r = capBuf[6];
        uint8_t g = capBuf[7];
        uint8_t b = capBuf[8];

        DialSlot slot = (DialSlot)slot_from_fa(slot_id);
        if (slot < DIAL_SLOT_COUNT) {
            Serial.printf(
                "[UI] live color slot=%d #%02X%02X%02X\n",
                slot, r, g, b
            );
            master_dial_set_color(slot, r, g, b);
        }
        
        return;
    }

    
    if (type == 0xFD && capBuf[4] == 0x33) {
        uint8_t reason = capBuf[5];

        Serial.printf("[HELIX] FD obj=33 reason=%02X\n", reason);

        // --- FD: config change → full resync required ---
        if (reason == 0x02) {
            Serial.println("[HELIX] DSP requests full resync");

            handshakeLocked = false;     // ← force unlock
            handshakeInProgress = false; // ← allow restart
            resetHandshake();
            return;
        }
    }

    printHex("[RX]", capBuf, capLen);

    // --- Always decode config/state ---
    if (type == 0xAF) {
        Serial.println("[DBG] AF blob received");
        decode_volume_blob(capBuf);
    }
    
    if (type == 0xF9 && capBuf[4] == 0x2A && capBuf[5] == 0x04) {
        decode_volume_snapshot(capBuf);
    }

    // --- Handshake FSM stops permanently after READY ---
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
            sendHS(HS7, sizeof(HS7), "HS7");
            hsState = HS_WAIT_FINAL;
        }
        break;

    case HS_WAIT_FINAL:
        if (type == 0xFB && capBuf[4] == 0x2B) {
            hs7AckMs = millis();

            ready = true;                 // ← CRITICAL
            handshakeInProgress = false;  // handshake is functionally done

            hsState = HS_WAIT_FD_CHALLENGE;
            Serial.println("[HELIX] HS7 ACK, READY asserted");
        }
        break;

    case HS_WAIT_FD_CHALLENGE:
        if (type == 0xFD && capBuf[4] == 0x33) {
            dsp->write(HS_FD_REPLY, sizeof(HS_FD_REPLY));
            dsp->flush();
            Serial.println("[HELIX] FD challenge answered");
           handshakeLocked = true;
            handshakeInProgress = false;
            hsState = HS_READY;
            Serial.println("[HELIX] READY (ownership latched)");
            break;
        }

        // No FD challenge → assume ownership
        if (millis() - hs7AckMs > 100) {   // 50–100 ms is plenty
            handshakeLocked = true;
            handshakeInProgress = false;
            hsState = HS_READY;
            Serial.println("[HELIX] READY (no FD challenge)");
        }
        break;

        default:
        break;
    }
}

// ================= PUBLIC API =================

void helix_begin(HardwareSerial& dspSerial)
{
    dsp = &dspSerial;
    resetHandshake();
    Serial.println("Boot Handshake");
}

bool helix_ready()
{
    return ready && volume[activeSlot].valid;
}

// ================= LOOP =================

void helix_loop()
{
    if (!dsp) return;

    uint32_t nowUs = micros();

    while (dsp->available()) {
        uint8_t b = dsp->read();

        lastDspRxMs = millis();

        if (capLen < FRAME_MAX)
            capBuf[capLen++] = b;

        lastByteUs = nowUs;
    }

    if (capLen && (nowUs - lastByteUs) > GAP_US) {
        processFrame();
        capLen = 0;
    }
}

// ================= VOLUME DELTA =================

void helix_volume_delta(int8_t clicks)
{
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

    uint8_t volCode = VOL_MIN + vm.index;

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
    master_dial_set_absolute(ui);
}

