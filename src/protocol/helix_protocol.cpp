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
    uint8_t steps;
    float   range_dB;
    float   step_dB;
    uint8_t index;
    bool    valid;
};

static VolumeModel vol1 = {0};

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

// ================= HANDSHAKE RESET =================

static void resetHandshake()
{
    Serial.println("[HELIX] handshake reset");

    ready = false;
    handshakeLocked = false;
    handshakeInProgress = true;
    vol1.valid = false;
    capLen = 0;
    hsState = HS_WAIT_ACK0;

    sendHS(HS0, sizeof(HS0), "HS0");
}

// ================= DECODERS =================

static void decode_volume_blob(const uint8_t *buf)
{
    const int o = 22;   // slot 1 base

    uint8_t steps = buf[o + 4];
    if (!steps) return;

    float range_dB = u16le(&buf[o + 6]) / 10.0f;

    vol1.steps   = steps;
    vol1.range_dB = range_dB;
    vol1.step_dB  = range_dB / steps;
    vol1.valid    = true;

    Serial.printf("[VOL] steps=%u range=%.1f dB step=%.2f dB\n",
                  vol1.steps, vol1.range_dB, vol1.step_dB);
}

static void decode_volume_snapshot(const uint8_t *buf)
{
    if (!vol1.valid) return;

    uint8_t idx = buf[6];
    if (idx > vol1.steps) idx = vol1.steps;

    vol1.index = idx;

    int ui = map(vol1.index, 0, vol1.steps, 0, 100);
    master_dial_set_absolute(ui);
}

// ================= FRAME PROCESSOR =================

static void processFrame()
{
    uint8_t type = capBuf[2];

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
    if (type == 0xAF)
        Serial.println("[DBG] AF blob received");
        decode_volume_blob(capBuf);
    
    if (type == 0xF9 && capBuf[4] == 0x2A && capBuf[5] == 0x04)
        decode_volume_snapshot(capBuf);

    // --- Handshake FSM stops permanently after READY ---
    if (handshakeLocked)
        return;

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
    return ready && vol1.valid;
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
    if (!helix_ready())
        return;

    int next = (int)vol1.index + clicks;

    if (next < 0) next = 0;
    if (next > vol1.steps) next = vol1.steps;
    if (next == vol1.index) return;

    vol1.index = (uint8_t)next;

    uint8_t volCode = VOL_MIN + vol1.index;

    const uint8_t wake[] = {0x42, 0x06};
    dsp->write(wake, sizeof(wake));
    dsp->flush();
    delayMicroseconds(FRAME_DELAY_US);

    uint8_t body[] = {
        0xF9, 0x01, 0x2B, 0x04, 0x00,
        vol1.index, 0x01, volCode
    };

    printHex("[TX VOL]", body, sizeof(body));
    dsp->write(body, sizeof(body));
    dsp->flush();

    int ui = map(vol1.index, 0, vol1.steps, 0, 100);
    master_dial_set_absolute(ui);
}
