#include "settings.h"
#include <Preferences.h>

static Preferences prefs;
static bool settings_ready = false;
static UiTheme current_theme = THEME_BASIC;

void settings_init()
{
    if (settings_ready)
        return;

    prefs.begin("conductor", false);
    current_theme = (UiTheme)prefs.getUChar("theme", (uint8_t)THEME_BASIC);
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
