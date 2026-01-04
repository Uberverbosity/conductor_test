#pragma once
#include <lvgl.h>
#include "protocol/helix_protocol.h"

void tone_page_create(lv_obj_t* parent, ToneBand band);
void tone_page_on_enter(ToneBand band);
void tone_page_delta(ToneBand band, int delta);
void tone_page_refresh(void);
