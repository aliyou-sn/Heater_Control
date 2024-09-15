#include "ui.h"

void ui_Screen2_screen_init(void)
{
ui_Screen2 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen2, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLL_ELASTIC );    /// Flags
lv_obj_set_scrollbar_mode(ui_Screen2, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x070606), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GrillSwitch = lv_switch_create(ui_Screen2);
lv_obj_set_width( ui_GrillSwitch, 220);
lv_obj_set_height( ui_GrillSwitch, 120);
lv_obj_set_x( ui_GrillSwitch, -65 );
lv_obj_set_y( ui_GrillSwitch, -55 );
lv_obj_set_align( ui_GrillSwitch, LV_ALIGN_CENTER );
lv_obj_set_style_bg_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_GrillSwitch, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_grad_dir(ui_GrillSwitch, LV_GRAD_DIR_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_img_recolor(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_img_recolor_opa(ui_GrillSwitch, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_outline_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_outline_opa(ui_GrillSwitch, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_anim_time(ui_GrillSwitch, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_radius(ui_GrillSwitch, 20, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_GrillSwitch, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_INDICATOR | LV_STATE_DEFAULT );

lv_obj_set_style_radius(ui_GrillSwitch, 10, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_GrillSwitch, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_main_stop(ui_GrillSwitch, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_stop(ui_GrillSwitch, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_GrillSwitch, lv_color_hex(0x51BCEF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_GrillSwitch, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_GrillSwitch, 0, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_blend_mode(ui_GrillSwitch, LV_BLEND_MODE_NORMAL, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_opa(ui_GrillSwitch, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_Switch2 = lv_switch_create(ui_Screen2);
lv_obj_set_width( ui_Switch2, 220);
lv_obj_set_height( ui_Switch2, 120);
lv_obj_set_x( ui_Switch2, -64 );
lv_obj_set_y( ui_Switch2, 83 );
lv_obj_set_align( ui_Switch2, LV_ALIGN_CENTER );
lv_obj_set_style_radius(ui_Switch2, 20, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Switch2, lv_color_hex(0x51BCEF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Switch2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Switch2, lv_color_hex(0x51BCEF), LV_PART_MAIN | LV_STATE_DEFAULT );

lv_obj_set_style_radius(ui_Switch2, 20, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Switch2, lv_color_hex(0x51BCEF), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Switch2, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Switch2, lv_color_hex(0x51BCEF), LV_PART_INDICATOR | LV_STATE_DEFAULT );

lv_obj_set_style_radius(ui_Switch2, 20, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Switch2, lv_color_hex(0x51BCEF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Switch2, 255, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_blend_mode(ui_Switch2, LV_BLEND_MODE_NORMAL, LV_PART_KNOB| LV_STATE_DEFAULT);
lv_obj_set_style_opa(ui_Switch2, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_Label2 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 2
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 2
lv_obj_set_x( ui_Label2, 136 );
lv_obj_set_y( ui_Label2, 82 );
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"WARMER");
lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label1 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 2
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 2
lv_obj_set_x( ui_Label1, 114 );
lv_obj_set_y( ui_Label1, -53 );
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"GRILL");
lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label3 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label3, -16 );
lv_obj_set_y( ui_Label3, -54 );
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"OFF");
lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label4 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label4, -16 );
lv_obj_set_y( ui_Label4, 84 );
lv_obj_set_align( ui_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label4,"OFF");
lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_30, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GrillIndicator = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_GrillIndicator, 94);
lv_obj_set_height( ui_GrillIndicator, 97);
lv_obj_set_x( ui_GrillIndicator, -115 );
lv_obj_set_y( ui_GrillIndicator, -57 );
lv_obj_set_align( ui_GrillIndicator, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_GrillIndicator, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_GrillIndicator, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_GrillIndicator, 60, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_GrillIndicator, lv_color_hex(0xECB520), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_GrillIndicator, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_WarmerIndicator = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_WarmerIndicator, 94);
lv_obj_set_height( ui_WarmerIndicator, 97);
lv_obj_set_x( ui_WarmerIndicator, -111 );
lv_obj_set_y( ui_WarmerIndicator, 84 );
lv_obj_set_align( ui_WarmerIndicator, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_WarmerIndicator, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_WarmerIndicator, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_WarmerIndicator, 60, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_WarmerIndicator, lv_color_hex(0xECB520), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_WarmerIndicator, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_GrillSwitch, ui_event_GrillSwitch, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Switch2, ui_event_Switch2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Label3, ui_event_Label3, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Screen2, ui_event_Screen2, LV_EVENT_ALL, NULL);

}
