#include "ui.h"

void ui_Screen3_screen_init(void)
{
ui_Screen3 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Screen3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Screen3, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_Screen3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );

ui_TimerPanel = lv_obj_create(ui_Screen3);
lv_obj_set_width( ui_TimerPanel, 91);
lv_obj_set_height( ui_TimerPanel, 39);
lv_obj_set_x( ui_TimerPanel, -168 );
lv_obj_set_y( ui_TimerPanel, -128 );
lv_obj_set_align( ui_TimerPanel, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_TimerPanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_TimerPanel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_TimerPanel, lv_color_hex(0x21824D), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_TimerPanel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_grad_color(ui_TimerPanel, lv_color_hex(0x21824D), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_color(ui_TimerPanel, lv_color_hex(0x21824D), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_TimerPanel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_MinutesLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_MinutesLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_MinutesLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_MinutesLabel, -191 );
lv_obj_set_y( ui_MinutesLabel, -127 );
lv_obj_set_align( ui_MinutesLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_MinutesLabel,"00");
lv_obj_set_style_text_color(ui_MinutesLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_MinutesLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_MinutesLabel, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SemiColon = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_SemiColon, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SemiColon, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SemiColon, -168 );
lv_obj_set_y( ui_SemiColon, -129 );
lv_obj_set_align( ui_SemiColon, LV_ALIGN_CENTER );
lv_label_set_text(ui_SemiColon,":");
lv_obj_set_style_text_color(ui_SemiColon, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SemiColon, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SemiColon, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_SemiColon, lv_color_hex(0x21824D), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SemiColon, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SecondsLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_SecondsLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SecondsLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SecondsLabel, -144 );
lv_obj_set_y( ui_SecondsLabel, -128 );
lv_obj_set_align( ui_SecondsLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SecondsLabel,"00");
lv_obj_set_style_text_color(ui_SecondsLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SecondsLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SecondsLabel, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_TempLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_TempLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_TempLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_TempLabel, -140 );
lv_obj_set_y( ui_TempLabel, -70 );
lv_obj_set_align( ui_TempLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_TempLabel,"TEMPERATURE");
lv_obj_set_style_text_color(ui_TempLabel, lv_color_hex(0x57C0AC), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_TempLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_letter_space(ui_TempLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_line_space(ui_TempLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_TempLabel, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GTempArea = lv_btn_create(ui_Screen3);
lv_obj_set_width( ui_GTempArea, 100);
lv_obj_set_height( ui_GTempArea, 64);
lv_obj_set_x( ui_GTempArea, -165 );
lv_obj_set_y( ui_GTempArea, 0 );
lv_obj_set_align( ui_GTempArea, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_GTempArea, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_GTempArea, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_GTempArea, 15, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_GTempArea, lv_color_hex(0xC4A736), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_GTempArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_WTempAre = lv_btn_create(ui_Screen3);
lv_obj_set_width( ui_WTempAre, 100);
lv_obj_set_height( ui_WTempAre, 64);
lv_obj_set_x( ui_WTempAre, -165 );
lv_obj_set_y( ui_WTempAre, 91 );
lv_obj_set_align( ui_WTempAre, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_WTempAre, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_WTempAre, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_WTempAre, 15, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_WTempAre, lv_color_hex(0xC4A736), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_WTempAre, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GSetTempArea = lv_btn_create(ui_Screen3);
lv_obj_set_width( ui_GSetTempArea, 100);
lv_obj_set_height( ui_GSetTempArea, 64);
lv_obj_set_x( ui_GSetTempArea, 106 );
lv_obj_set_y( ui_GSetTempArea, 0 );
lv_obj_set_align( ui_GSetTempArea, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_GSetTempArea, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_GSetTempArea, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_GSetTempArea, 15, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_GSetTempArea, lv_color_hex(0xC4A736), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_GSetTempArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_WSetTempArea = lv_btn_create(ui_Screen3);
lv_obj_set_width( ui_WSetTempArea, 100);
lv_obj_set_height( ui_WSetTempArea, 64);
lv_obj_set_x( ui_WSetTempArea, 106 );
lv_obj_set_y( ui_WSetTempArea, 91 );
lv_obj_set_align( ui_WSetTempArea, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_WSetTempArea, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_WSetTempArea, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_WSetTempArea, 15, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_WSetTempArea, lv_color_hex(0xC4A736), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_WSetTempArea, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_SetLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_SetLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_SetLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_SetLabel, 108 );
lv_obj_set_y( ui_SetLabel, -72 );
lv_obj_set_align( ui_SetLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_SetLabel,"SET");
lv_obj_set_style_text_color(ui_SetLabel, lv_color_hex(0x57C0AC), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_SetLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_letter_space(ui_SetLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_line_space(ui_SetLabel, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_SetLabel, &lv_font_montserrat_22, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_GLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_GLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_GLabel, -94 );
lv_obj_set_y( ui_GLabel, -1 );
lv_obj_set_align( ui_GLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_GLabel,"G");
lv_obj_set_style_text_color(ui_GLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_GLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_GLabel, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_WLAbel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_WLAbel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_WLAbel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_WLAbel, -90 );
lv_obj_set_y( ui_WLAbel, 87 );
lv_obj_set_align( ui_WLAbel, LV_ALIGN_CENTER );
lv_label_set_text(ui_WLAbel,"W");
lv_obj_set_style_text_color(ui_WLAbel, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_WLAbel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_WLAbel, &lv_font_montserrat_26, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_CelsiusFareinheitLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_CelsiusFareinheitLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_CelsiusFareinheitLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_CelsiusFareinheitLabel, -64 );
lv_obj_set_y( ui_CelsiusFareinheitLabel, 45 );
lv_obj_set_align( ui_CelsiusFareinheitLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_CelsiusFareinheitLabel,"°C");
lv_obj_set_style_text_color(ui_CelsiusFareinheitLabel, lv_color_hex(0xDAF913), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_CelsiusFareinheitLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_CelsiusFareinheitLabel, &lv_font_montserrat_36, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GCurrentTempLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_GCurrentTempLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_GCurrentTempLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_GCurrentTempLabel, -165 );
lv_obj_set_y( ui_GCurrentTempLabel, 4 );
lv_obj_set_align( ui_GCurrentTempLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_GCurrentTempLabel,"70");
lv_obj_set_style_text_color(ui_GCurrentTempLabel, lv_color_hex(0x010000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_GCurrentTempLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_GCurrentTempLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_WCurrentTempLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_WCurrentTempLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_WCurrentTempLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_WCurrentTempLabel, -165 );
lv_obj_set_y( ui_WCurrentTempLabel, 92 );
lv_obj_set_align( ui_WCurrentTempLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_WCurrentTempLabel,"120");
lv_obj_set_style_text_color(ui_WCurrentTempLabel, lv_color_hex(0x010000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_WCurrentTempLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_WCurrentTempLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_GSetTempLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_GSetTempLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_GSetTempLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_GSetTempLabel, 106 );
lv_obj_set_y( ui_GSetTempLabel, 2 );
lv_obj_set_align( ui_GSetTempLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_GSetTempLabel,"200");
lv_obj_set_style_text_color(ui_GSetTempLabel, lv_color_hex(0x010000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_GSetTempLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_GSetTempLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_WSetTempLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_WSetTempLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_WSetTempLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_WSetTempLabel, 106 );
lv_obj_set_y( ui_WSetTempLabel, 90 );
lv_obj_set_align( ui_WSetTempLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_WSetTempLabel,"85");
lv_obj_set_style_text_color(ui_WSetTempLabel, lv_color_hex(0x010000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_WSetTempLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_WSetTempLabel, &lv_font_montserrat_24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ImgButton3 = lv_imgbtn_create(ui_Screen3);
lv_imgbtn_set_src(ui_ImgButton3, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1919573283, NULL);
lv_obj_set_height( ui_ImgButton3, 50);
lv_obj_set_width( ui_ImgButton3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_ImgButton3, 197 );
lv_obj_set_y( ui_ImgButton3, -125 );
lv_obj_set_align( ui_ImgButton3, LV_ALIGN_CENTER );

ui_ImgButton5 = lv_imgbtn_create(ui_Screen3);
lv_imgbtn_set_src(ui_ImgButton5, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1658756774, NULL);
lv_obj_set_height( ui_ImgButton5, 42);
lv_obj_set_width( ui_ImgButton5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_ImgButton5, 31 );
lv_obj_set_y( ui_ImgButton5, 3 );
lv_obj_set_align( ui_ImgButton5, LV_ALIGN_CENTER );

ui_GMinusLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_GMinusLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_GMinusLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_GMinusLabel, 21 );
lv_obj_set_y( ui_GMinusLabel, -2 );
lv_obj_set_align( ui_GMinusLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_GMinusLabel,"-");
lv_obj_set_style_text_color(ui_GMinusLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_GMinusLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_GMinusLabel, &lv_font_montserrat_38, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ImgButton1 = lv_imgbtn_create(ui_Screen3);
lv_imgbtn_set_src(ui_ImgButton1, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1658756774, NULL);
lv_obj_set_height( ui_ImgButton1, 42);
lv_obj_set_width( ui_ImgButton1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_ImgButton1, 31 );
lv_obj_set_y( ui_ImgButton1, 92 );
lv_obj_set_align( ui_ImgButton1, LV_ALIGN_CENTER );

ui_WMinusLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_WMinusLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_WMinusLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_WMinusLabel, 19 );
lv_obj_set_y( ui_WMinusLabel, 87 );
lv_obj_set_align( ui_WMinusLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_WMinusLabel,"-");
lv_obj_set_style_text_color(ui_WMinusLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_WMinusLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_WMinusLabel, &lv_font_montserrat_38, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ImgButton9 = lv_imgbtn_create(ui_Screen3);
lv_imgbtn_set_src(ui_ImgButton9, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1217371524, NULL);
lv_obj_set_height( ui_ImgButton9, 50);
lv_obj_set_width( ui_ImgButton9, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_ImgButton9, 183 );
lv_obj_set_y( ui_ImgButton9, 2 );
lv_obj_set_align( ui_ImgButton9, LV_ALIGN_CENTER );

ui_GPlusLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_GPlusLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_GPlusLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_GPlusLabel, 190 );
lv_obj_set_y( ui_GPlusLabel, 0 );
lv_obj_set_align( ui_GPlusLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_GPlusLabel,"+");
lv_obj_set_style_text_color(ui_GPlusLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_GPlusLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_GPlusLabel, &lv_font_montserrat_38, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_ImgButton2 = lv_imgbtn_create(ui_Screen3);
lv_imgbtn_set_src(ui_ImgButton2, LV_IMGBTN_STATE_RELEASED, NULL, &ui_img_1217371524, NULL);
lv_obj_set_height( ui_ImgButton2, 50);
lv_obj_set_width( ui_ImgButton2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_ImgButton2, 185 );
lv_obj_set_y( ui_ImgButton2, 92 );
lv_obj_set_align( ui_ImgButton2, LV_ALIGN_CENTER );

ui_WPlusLabel = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_WPlusLabel, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_WPlusLabel, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_WPlusLabel, 192 );
lv_obj_set_y( ui_WPlusLabel, 91 );
lv_obj_set_align( ui_WPlusLabel, LV_ALIGN_CENTER );
lv_label_set_text(ui_WPlusLabel,"+");
lv_obj_set_style_text_color(ui_WPlusLabel, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_WPlusLabel, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_WPlusLabel, &lv_font_montserrat_38, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_ImgButton3, ui_event_ImgButton3, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ImgButton5, ui_event_ImgButton5, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ImgButton1, ui_event_ImgButton1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ImgButton9, ui_event_ImgButton9, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_ImgButton2, ui_event_ImgButton2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Screen3, ui_event_Screen3, LV_EVENT_ALL, NULL);

}
