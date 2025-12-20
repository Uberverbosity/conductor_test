#include "helix_protocol.h"

static HardwareSerial* dsp = nullptr;
static bool ready = false;

// TEMP until blob parsed
static int masterIndex = 60;
static int masterSteps = 60;
static float stepDb = 0.5f;

// Handshake packets
static const uint8_t HS0[] = {0x42,0x03,0xFC,0x01,0x2A,0x00,0x2A};
static const uint8_t HS1[] = {0x42,0x03,0xFC,0x01,0x2A,0x03,0x2D};

void helix_begin(HardwareSerial& dspSerial)
{
    dsp = &dspSerial;
    ready = false;

    Serial.println("[HELIX] starting handshake");
    dsp->write(HS0, sizeof(HS0));
}

void helix_loop()
{
    while (dsp->available()) {
        uint8_t b = dsp->read();
        Serial.printf("%02X ", b);

        // crude markers for now
        if (b == 0xAF) {
            Serial.println("\n[HELIX] blob detected");
        }
        if (b == 0xFB) {
            ready = true;
            Serial.println("\n[HELIX] READY");
        }
    }
}

bool helix_ready()
{
    return ready;
}

void helix_volume_delta(int8_t clicks)
{
    if (!ready) {
        Serial.println("[HELIX] volume ignored (not ready)");
        return;
    }

    masterIndex += clicks;
    if (masterIndex < 0) masterIndex = 0;
    if (masterIndex > masterSteps * 2)
        masterIndex = masterSteps * 2;

    uint8_t pkt[] = {
        0x42,0x06,0xF9,
        0x01,0x2A,0x04,
        (uint8_t)masterIndex,
        0x00
    };

    uint8_t sum = 0;
    for (int i = 0; i < sizeof(pkt) - 1; i++)
        sum += pkt[i];
    pkt[sizeof(pkt) - 1] = sum;

    dsp->write(pkt, sizeof(pkt));

    Serial.printf(
        "[VOL] idx=%d  db=%.1f\n",
        masterIndex,
        (masterIndex - masterSteps) * stepDb
    );
}
