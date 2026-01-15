#pragma once
#include <stdint.h>

enum UiTheme : uint8_t {
    THEME_BASIC = 0,
    THEME_MCINTOSH = 1
};

void settings_init();
UiTheme settings_get_theme();
void settings_set_theme(UiTheme theme);
