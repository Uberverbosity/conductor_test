#include "settings.h"
#include <Preferences.h>

static Preferences prefs;
static bool settings_ready = false;
static UiTheme current_theme = THEME_BASIC;
static MasterStepSize current_step_size = STEP_SIZE_1_0_DB;

void settings_init()
{
    if (settings_ready)
        return;

    prefs.begin("conductor", false);
    current_theme = (UiTheme)prefs.getUChar("theme", (uint8_t)THEME_BASIC);
    current_step_size = (MasterStepSize)prefs.getUChar("step_size", (uint8_t)STEP_SIZE_1_0_DB);
    settings_ready = true;
}

UiTheme settings_get_theme()
{
    if (!settings_ready)
        settings_init();
    return current_theme;
}

void settings_set_theme(UiTheme theme)
{
    if (!settings_ready)
        settings_init();

    if (current_theme == theme)
        return;

    current_theme = theme;
    prefs.putUChar("theme", (uint8_t)current_theme);
}

MasterStepSize settings_get_master_step_size()
{
    if (!settings_ready)
        settings_init();
    return current_step_size;
}

void settings_set_master_step_size(MasterStepSize step_size)
{
    if (!settings_ready)
        settings_init();

    if (current_step_size == step_size)
        return;

    current_step_size = step_size;
    prefs.putUChar("step_size", (uint8_t)current_step_size);
}

float settings_get_master_step_size_dB()
{
    MasterStepSize size = settings_get_master_step_size();
    switch (size) {
        case STEP_SIZE_0_5_DB: return 0.5f;
        case STEP_SIZE_1_0_DB: return 1.0f;
        case STEP_SIZE_2_0_DB: return 2.0f;
        case STEP_SIZE_3_0_DB: return 3.0f;
        case STEP_SIZE_4_0_DB: return 4.0f;
        default: return 1.0f;
    }
}
