#pragma once

// USER DEFINED SETTINGS (Merged for ESP32-C3 + TFT_eSPI v2.5.43 + LVGL v9.4.0)
// Based on official User_Setup template; custom LVGL + pins/freq/dims.

// Suppress warnings (harmless for C3)
#define TOUCH_CS -1  // No touch screen

// Patch for ESP32-C3 HAL quirk (core 2.0.14): Manual TFT command defines (fixes compile errors)
#define TFT_SWRST    0x01  // Software reset
#define TFT_INIT_DELAY 0x80  // Delay flag for init sequences
#define TFT_CASET    0x2A  // Column address set
#define TFT_PASET    0x2B  // Page (row) address set
#define TFT_RAMWR    0x2C  // Memory write
#define TFT_RAMRD    0x2E  // Memory read
#define TFT_INVON    0x21  // Display invert on
#define TFT_INVOFF   0x20  // Display invert off

#define CGRAM_OFFSET          // Required for GC9A01 round displays
#define TFT_RGB_ORDER TFT_RGB // Required (panel uses BGR)
#define TFT_INVERSION_ON      // Required or the screen stays dim/black

// Your custom LVGL integration
#define LV_LVGL_TFT_ESPI_IMPLEMENTATION  // Enables LVGL flush callback for TFT_eSPI

// Section 1: Call up the right driver (from template; only one uncommented)
//#define ILI9341_DRIVER  // Generic for 240x240 ILI9341; change to ST7789_DRIVER if needed
#undef ST7789_DRIVER  // Alternative for ST7789/GC9A01
#define GC9A01_DRIVER

// Section 2: Pins for ESP32-C3 (hardware SPI; your custom values)
#define TFT_MISO -1  // Not used (no readback for LVGL direct mode)
#define TFT_MOSI 0   // Your custom MOSI
#define TFT_SCLK 1   // Your custom SCLK
#define TFT_CS   10  // Your custom CS
#define TFT_DC   4   // Your custom DC
#define TFT_RST  2   // Your custom RST
#define TFT_BL  -1   // Your custom Backlight

// Your custom dimensions
#define TFT_WIDTH  240
#define TFT_HEIGHT 240

// Section 3: Fonts (minimal from template; for LVGL labels)
#define LOAD_GLCD   // Font 1 (8px basic)
#define LOAD_FONT2  // Font 2 (16px small)
#define LOAD_FONT4  // Font 4 (26px medium)
#define SMOOTH_FONT // Anti-aliased for LVGL

// Section 4: Other options (your custom SPI freq; template defaults)
#define SPI_FREQUENCY  40000000  // Your custom 40MHz (safe for C3/ILI9341)
#define SPI_READ_FREQUENCY 20000000  // Template default for reads
//#define SUPPORT_TRANSACTIONS  // Enabled by default on ESP32

// Enable custom setup (required)
#define USER_SETUP_LOADED 1
#define USER_SETUP_INFO "ESP32-C3_LVGL_240x240"