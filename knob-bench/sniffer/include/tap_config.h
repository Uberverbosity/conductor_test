#pragma once

// Defaults live in platformio.ini so the connector pins are visible in one place.
#ifndef TAP_RX_K2D
#define TAP_RX_K2D 4
#endif
#ifndef TAP_RX_D2K
#define TAP_RX_D2K 5
#endif
#ifndef TAP_BAUD
#define TAP_BAUD 230400
#endif

// UART RX-idle threshold, in bit times. 70 bits at 230400 is ~304 µs, just
// past the knob's 300 µs frame gap and well under the 600 µs volume-wake gap.
#ifndef TAP_RX_TOUT_BITS
#define TAP_RX_TOUT_BITS 70
#endif

#ifndef TAP_FRAME_MAX
#define TAP_FRAME_MAX 512
#endif

#if TAP_RX_K2D == 19 || TAP_RX_K2D == 20 || TAP_RX_D2K == 19 || TAP_RX_D2K == 20
#error Listen pins collide with ESP32-S3 native USB on GPIO 19 and 20
#endif
