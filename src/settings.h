#pragma once
#include <stdint.h>

enum UiTheme : uint8_t {
    THEME_BASIC = 0,
    THEME_MCINTOSH = 1
};

enum MasterStepSize : uint8_t {
    STEP_SIZE_0_5_DB = 0,
    STEP_SIZE_1_0_DB = 1,
    STEP_SIZE_2_0_DB = 2,
    STEP_SIZE_3_0_DB = 3,
    STEP_SIZE_4_0_DB = 4
};

void settings_init();
UiTheme settings_get_theme();
void settings_set_theme(UiTheme theme);
MasterStepSize settings_get_master_step_size();
void settings_set_master_step_size(MasterStepSize step_size);
float settings_get_master_step_size_dB();
