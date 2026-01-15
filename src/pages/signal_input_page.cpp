#include "signal_input_page.h"
#include <Arduino.h>
#include <lvgl.h>
#include <cstdio>

// ================= INTERNAL UI OBJECTS =================

static lv_obj_t* list_container = nullptr;
static lv_obj_t* list_labels[4] = {nullptr, nullptr, nullptr, nullptr};

// ================= STYLING =================

static const lv_color_t BG_COLOR = lv_color_hex(0x000000);
static const lv_color_t SELECTED_COLOR = lv_color_hex(0xFFFFFF);  // Bright white
static const lv_color_t UNSELECTED_COLOR = lv_color_hex(0x777777); // Dimmer gray

// List items
static const char* INPUT_NAMES[] = {
    "RCA (preamp)",
    "high-level (speaker)",
    "digital (optical)",
    "digital (coaxial)"
};

// ================= STATE =================

static int selected_index = 0;  // Currently selected item (0-3)

// ================= INTERNAL HELPERS =================

static void update_list_display()
{
    if (!list_container)
        return;

    // Calculate vertical spacing
    const int LABEL_SPACING = 40;  // Vertical spacing between list items
    
    for (int i = 0; i < 4; i++) {
        if (!list_labels[i])
            continue;
        
        // Calculate position: center item at screen center, others offset
        int offset = (i - selected_index) * LABEL_SPACING;
        
        // Set label position (centered horizontally, offset vertically)
        lv_obj_align(list_labels[i], LV_ALIGN_CENTER, 0, offset);
        
        // Update color based on selection
        lv_color_t color = (i == selected_index) ? SELECTED_COLOR : UNSELECTED_COLOR;
        lv_obj_set_style_text_color(list_labels[i], color, 0);
        
        // Make selected item slightly larger/bolder by adjusting opacity or using larger font
        // For now, just use color difference
    }
}

// ================= PUBLIC API =================

void signal_input_page_create(lv_obj_t* parent)
{
    list_container = parent;
    
    lv_obj_set_style_bg_color(parent, BG_COLOR, 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    
    // Create labels for each input option
    for (int i = 0; i < 4; i++) {
        list_labels[i] = lv_label_create(parent);
        
        lv_label_set_text(list_labels[i], INPUT_NAMES[i]);
        
        // Center horizontally
        lv_obj_set_style_text_align(list_labels[i], LV_TEXT_ALIGN_CENTER, 0);
        
        // Use a readable font size (similar to other pages)
        lv_obj_set_style_text_font(list_labels[i], &lv_font_montserrat_20, 0);
        
        // Set initial color (will be updated by update_list_display)
        lv_obj_set_style_text_color(list_labels[i], UNSELECTED_COLOR, 0);
        
        // Center the label initially
        lv_obj_center(list_labels[i]);
    }
    
    // Initial display update
    selected_index = 0;
    update_list_display();
}

void signal_input_page_on_enter(void)
{
    // Reset to first item
    selected_index = 0;
    update_list_display();
}

void signal_input_page_refresh(void)
{
    update_list_display();
}

void signal_input_page_delta(int delta)
{
    int next = selected_index + delta;
    
    // Clamp to valid range
    if (next < 0) next = 0;
    if (next > 3) next = 3;
    
    if (next == selected_index)
        return;
    
    selected_index = next;
    update_list_display();
}
