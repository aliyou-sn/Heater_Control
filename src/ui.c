#include <Arduino.h>
#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1( lv_event_t * e);
lv_obj_t *ui_Screen1;
lv_obj_t *ui_Image1;
lv_obj_t *ui_Image2;


// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
void ui_event_Screen2( lv_event_t * e);
lv_obj_t *ui_Screen2;
void ui_event_GrillSwitch( lv_event_t * e);
lv_obj_t *ui_GrillSwitch;
void ui_event_Switch2( lv_event_t * e);
lv_obj_t *ui_Switch2;
lv_obj_t *ui_Label2;
lv_obj_t *ui_Label1;
void ui_event_Label3( lv_event_t * e);
lv_obj_t *ui_Label3;
lv_obj_t *ui_Label4;
lv_obj_t *ui_GrillIndicator;
lv_obj_t *ui_WarmerIndicator;


// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
void ui_event_Screen3( lv_event_t * e);
lv_obj_t *ui_Screen3;
lv_obj_t *ui_TimerPanel;
lv_obj_t *ui_MinutesLabel;
lv_obj_t *ui_SemiColon;
lv_obj_t *ui_SecondsLabel;
lv_obj_t *ui_TempLabel;
lv_obj_t *ui_GTempArea;
lv_obj_t *ui_WTempAre;
lv_obj_t *ui_GSetTempArea;
lv_obj_t *ui_WSetTempArea;
lv_obj_t *ui_SetLabel;
lv_obj_t *ui_GLabel;
lv_obj_t *ui_WLAbel;
lv_obj_t *ui_CelsiusFareinheitLabel;
lv_obj_t *ui_GCurrentTempLabel;
lv_obj_t *ui_WCurrentTempLabel;
lv_obj_t *ui_GSetTempLabel;
lv_obj_t *ui_WSetTempLabel;
void ui_event_ImgButton3( lv_event_t * e);
lv_obj_t *ui_ImgButton3;
void ui_event_ImgButton5( lv_event_t * e);
lv_obj_t *ui_ImgButton5;
lv_obj_t *ui_GMinusLabel;
void ui_event_ImgButton1( lv_event_t * e);
lv_obj_t *ui_ImgButton1;
lv_obj_t *ui_WMinusLabel;
void ui_event_ImgButton9( lv_event_t * e);
lv_obj_t *ui_ImgButton9;
lv_obj_t *ui_GPlusLabel;
void ui_event_ImgButton2( lv_event_t * e);
lv_obj_t *ui_ImgButton2;
lv_obj_t *ui_WPlusLabel;


// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
lv_obj_t *ui_Screen4;
void ui_event_ImgButton4( lv_event_t * e);
lv_obj_t *ui_ImgButton4;
void ui_event_BrightnessSlider( lv_event_t * e);
lv_obj_t *ui_BrightnessSlider;
void ui_event_CelsiusFarenheitButton( lv_event_t * e);
lv_obj_t *ui_CelsiusFarenheitButton;
lv_obj_t *ui_LanguageButton;
lv_obj_t *ui_BrightnessLabel;
lv_obj_t *ui_Label13;
lv_obj_t *ui_Label5;
lv_obj_t *ui_CelsiusFarenheitLabel;
lv_obj_t *ui_LanguageLabel;
lv_obj_t *ui_BrightnessValueLabel;
lv_obj_t *ui____initial_actions0;
const lv_img_dsc_t *ui_imgset_logo_[1] = {&ui_img_logo_2_png};

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Screen1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_SCREEN_LOADED) {
      _ui_screen_change( &ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 5, 5000, &ui_Screen2_screen_init);
}
}
void ui_event_Screen2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
    
if ( event_code == LV_EVENT_CLICKED) {
     _ui_screen_change( &ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 5, 0, &ui_Screen3_screen_init);
}

}
void ui_event_GrillSwitch( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_checked_set_text_value( ui_Label3, target, "ON", "OFF");
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      Grill_On( e );
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      Grill_Off( e );
}
}
void ui_event_Switch2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_checked_set_text_value( ui_Label4, target, "ON", "OFF");
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      Warmer_On( e );
}
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      Warmer_Off( e );
      ( e );
}
}
void ui_event_Label3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_checked_set_text_value( ui_Label3, target, "ON", "OFF");
}
}
void ui_event_Screen3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
    //  _ui_screen_change( &ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 5, 0, &ui_Screen2_screen_init);
     _ui_screen_change( &ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 5, 0, &ui_Screen2_screen_init); 

}



}
void ui_event_ImgButton3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_screen_change( &ui_Screen4, LV_SCR_LOAD_ANIM_FADE_ON, 5, 0, &ui_Screen4_screen_init);
}
}
void ui_event_ImgButton5( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      DecrementGTemperature( e );
}
}
void ui_event_ImgButton1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      DecrementWTemperature( e );
}
}
void ui_event_ImgButton9( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      IncrementGTemperature( e );
}
}
void ui_event_ImgButton2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      IncrementWTemperature( e );
}
}
void ui_event_ImgButton4( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_screen_change( &ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 5, 0, &ui_Screen3_screen_init);
}
}
void ui_event_BrightnessSlider( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_VALUE_CHANGED) {
      _ui_slider_set_text_value( ui_BrightnessValueLabel, target, "", "%");
    int brightness = lv_slider_get_value(target);  // Get slider value (0-255)
    ledcWrite(PWM_CHANNEL, brightness);

}
}
void ui_event_CelsiusFarenheitButton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      SwapCelsiusFarenheit( e );
}
}

///////////////////// SCREENS ////////////////////

void ui_init( void )
{
lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);
ui_Screen1_screen_init();
ui_Screen2_screen_init();
ui_Screen3_screen_init();
ui_Screen4_screen_init();
ui____initial_actions0 = lv_obj_create(NULL);
lv_disp_load_scr( ui_Screen1);
}
