#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "tap_config.h"

static const size_t LOG_QUEUE_DEPTH = 24;

struct Frame {
    uint64_t t_us;
    char tag[4];
    uint16_t len;
    uint8_t trunc;
    uint8_t data[TAP_FRAME_MAX];
};

struct Tap {
    uart_port_t port;
    QueueHandle_t events;
    const char* tag;
    uint8_t data[TAP_FRAME_MAX];
    size_t len;
    bool trunc;
    bool open;
    uint64_t t0;
};

static Tap tapK2d = {};
static Tap tapD2k = {};
static QueueHandle_t logQueue = nullptr;
static volatile uint32_t dropCount = 0;

static bool installTap(Tap& tap, int rxPin) {
    uart_config_t cfg = {};
    cfg.baud_rate = TAP_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;

    esp_err_t err = uart_driver_install(tap.port, 4096, 0, 32, &tap.events, 0);
    if (err != ESP_OK) {
        Serial.printf("# uart %d install failed: %d\n", (int)tap.port, (int)err);
        return false;
    }
    uart_param_config(tap.port, &cfg);
    // TX pin is left unconnected. This firmware never transmits.
    uart_set_pin(tap.port, UART_PIN_NO_CHANGE, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_rx_timeout(tap.port, TAP_RX_TOUT_BITS);
    uart_set_rx_full_threshold(tap.port, 120);
    tap.open = true;
    return true;
}

static void noteDrop() {
    dropCount++;
}

static void emit(Tap& tap) {
    if (tap.len == 0 && !tap.trunc) {
        return;
    }
    Frame frame = {};
    frame.t_us = tap.t0;
    strncpy(frame.tag, tap.tag, sizeof(frame.tag) - 1);
    frame.len = (uint16_t)tap.len;
    frame.trunc = tap.trunc ? 1 : 0;
    if (tap.len > 0) {
        memcpy(frame.data, tap.data, tap.len);
    }
    if (xQueueSend(logQueue, &frame, 0) != pdTRUE) {
        noteDrop();
    }
    tap.len = 0;
    tap.trunc = false;
    tap.t0 = 0;
}

static void appendBytes(Tap& tap, const uint8_t* bytes, size_t n) {
    if (n == 0) {
        return;
    }
    if (tap.len == 0) {
        tap.t0 = esp_timer_get_time();
    }
    for (size_t i = 0; i < n; i++) {
        if (tap.len >= TAP_FRAME_MAX) {
            tap.trunc = true;
            return;
        }
        tap.data[tap.len++] = bytes[i];
    }
}

static void onUartEvent(Tap& tap, const uart_event_t& ev) {
    if (!tap.open) {
        return;
    }
    if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
        uart_flush_input(tap.port);
        xQueueReset(tap.events);
        tap.trunc = true;
        emit(tap);
        noteDrop();
        return;
    }
    if (ev.type != UART_DATA) {
        return;
    }

    size_t left = ev.size;
    uint8_t tmp[128];
    while (left > 0) {
        size_t chunk = left > sizeof(tmp) ? sizeof(tmp) : left;
        int n = uart_read_bytes(tap.port, tmp, chunk, pdMS_TO_TICKS(20));
        if (n <= 0) {
            break;
        }
        appendBytes(tap, tmp, (size_t)n);
        left -= (size_t)n;
    }
    // timeout_flag is set by the UART when the line has been idle for
    // TAP_RX_TOUT_BITS. Framing happens in the peripheral, so a late read
    // still sees the same gaps the knob uses.
    if (ev.timeout_flag) {
        emit(tap);
    }
}

static void tapTask(void*) {
    while (true) {
        bool got = false;
        uart_event_t ev;
        if (tapK2d.open && xQueueReceive(tapK2d.events, &ev, 0) == pdTRUE) {
            onUartEvent(tapK2d, ev);
            got = true;
        }
        if (tapD2k.open && xQueueReceive(tapD2k.events, &ev, 0) == pdTRUE) {
            onUartEvent(tapD2k, ev);
            got = true;
        }
        if (!got) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

static void logTask(void*) {
    uint32_t announced = 0;
    while (true) {
        Frame frame;
        if (xQueueReceive(logQueue, &frame, pdMS_TO_TICKS(200)) == pdTRUE) {
            char line[64 + TAP_FRAME_MAX * 3];
            int n = snprintf(
                line,
                sizeof(line),
                "%llu %s",
                (unsigned long long)frame.t_us,
                frame.tag
            );
            if (n < 0) {
                n = 0;
            }
            for (uint16_t i = 0; i < frame.len && n < (int)sizeof(line) - 8; i++) {
                static const char kHex[] = "0123456789ABCDEF";
                uint8_t b = frame.data[i];
                line[n++] = ' ';
                line[n++] = kHex[b >> 4];
                line[n++] = kHex[b & 0x0F];
            }
            if (frame.trunc && n < (int)sizeof(line) - 8) {
                memcpy(line + n, " TRUNC", 6);
                n += 6;
            }
            line[n++] = '\n';
            Serial.write(reinterpret_cast<const uint8_t*>(line), n);
        }
        uint32_t drops = dropCount;
        if (drops != announced) {
            Serial.printf("DROP %lu\n", (unsigned long)drops);
            announced = drops;
        }
    }
}

void setup() {
    Serial.begin(115200);
    // DTR from a host open must not reboot the tap.
    Serial.enableReboot(false);
    delay(200);

    const uint32_t gapUs = (uint32_t)((uint64_t)TAP_RX_TOUT_BITS * 1000000ULL / TAP_BAUD);
    Serial.printf(
        "# knob-bench sniffer baud=%d gap_us=%lu k2d_rx=%d d2k_rx=%d\n",
        TAP_BAUD,
        (unsigned long)gapUs,
        TAP_RX_K2D,
        TAP_RX_D2K
    );

    logQueue = xQueueCreate(LOG_QUEUE_DEPTH, sizeof(Frame));

    tapK2d.port = UART_NUM_1;
    tapK2d.tag = "K2D";
    tapD2k.port = UART_NUM_2;
    tapD2k.tag = "D2K";

    installTap(tapK2d, TAP_RX_K2D);
    installTap(tapD2k, TAP_RX_D2K);

    xTaskCreatePinnedToCore(tapTask, "tap", 8192, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(logTask, "log", 8192, nullptr, 4, nullptr, 1);
}

void loop() {
    delay(1000);
}
