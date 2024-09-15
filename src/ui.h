#ifndef _TFT_DISPLAY_UI_H
#define _TFT_DISPLAY_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
  #if __has_include("lvgl.h")
    #include "lvgl.h"
  #elif __has_include("lvgl/lvgl.h")
    #include "lvgl/lvgl.h"
  #else
    #include "lvgl.h"
  #endif
#else
  #include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"

// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1( lv_event_t * e);
extern lv_obj_t *ui_Screen1;
extern lv_obj_t *ui_Image1;
extern lv_obj_t *ui_Image2;
// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
void ui_event_Screen2( lv_event_t * e);
extern lv_obj_t *ui_Screen2;
void ui_event_GrillSwitch( lv_event_t * e);
extern lv_obj_t *ui_GrillSwitch;
void ui_event_Switch2( lv_event_t * e);
extern lv_obj_t *ui_Switch2;
extern lv_obj_t *ui_Label2;
extern lv_obj_t *ui_Label1;
void ui_event_Label3( lv_event_t * e);
extern lv_obj_t *ui_Label3;
extern lv_obj_t *ui_Label4;
extern lv_obj_t *ui_GrillIndicator;
extern lv_obj_t *ui_WarmerIndicator;
// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
void ui_event_Screen3( lv_event_t * e);
extern lv_obj_t *ui_Screen3;
extern lv_obj_t *ui_TimerPanel;
extern lv_obj_t *ui_MinutesLabel;
extern lv_obj_t *ui_SemiColon;
extern lv_obj_t *ui_SecondsLabel;
extern lv_obj_t *ui_TempLabel;
extern lv_obj_t *ui_GTempArea;
extern lv_obj_t *ui_WTempAre;
extern lv_obj_t *ui_GSetTempArea;
extern lv_obj_t *ui_WSetTempArea;
extern lv_obj_t *ui_SetLabel;
extern lv_obj_t *ui_GLabel;
extern lv_obj_t *ui_WLAbel;
extern lv_obj_t *ui_CelsiusFareinheitLabel;
extern lv_obj_t *ui_GCurrentTempLabel;
extern lv_obj_t *ui_WCurrentTempLabel;
extern lv_obj_t *ui_GSetTempLabel;
extern lv_obj_t *ui_WSetTempLabel;
void ui_event_ImgButton3( lv_event_t * e);
extern lv_obj_t *ui_ImgButton3;
void ui_event_ImgButton5( lv_event_t * e);
extern lv_obj_t *ui_ImgButton5;
extern lv_obj_t *ui_GMinusLabel;
void ui_event_ImgButton1( lv_event_t * e);
extern lv_obj_t *ui_ImgButton1;
extern lv_obj_t *ui_WMinusLabel;
void ui_event_ImgButton9( lv_event_t * e);
extern lv_obj_t *ui_ImgButton9;
extern lv_obj_t *ui_GPlusLabel;
void ui_event_ImgButton2( lv_event_t * e);
extern lv_obj_t *ui_ImgButton2;
extern lv_obj_t *ui_WPlusLabel;
// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
extern lv_obj_t *ui_Screen4;
void ui_event_ImgButton4( lv_event_t * e);
extern lv_obj_t *ui_ImgButton4;
void ui_event_BrightnessSlider( lv_event_t * e);
extern lv_obj_t *ui_BrightnessSlider;
void ui_event_CelsiusFarenheitButton( lv_event_t * e);
extern lv_obj_t *ui_CelsiusFarenheitButton;
extern lv_obj_t *ui_LanguageButton;
extern lv_obj_t *ui_BrightnessLabel;
extern lv_obj_t *ui_Label13;
extern lv_obj_t *ui_Label5;
extern lv_obj_t *ui_CelsiusFarenheitLabel;
extern lv_obj_t *ui_LanguageLabel;
extern lv_obj_t *ui_BrightnessValueLabel;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_432405692);   // assets/LOGO (1).png
LV_IMG_DECLARE( ui_img_1919573283);   // assets/settings_icom (1).png
LV_IMG_DECLARE( ui_img_1658756774);   // assets/Arrow (1).png
LV_IMG_DECLARE( ui_img_1217371524);   // assets/right_Arrow (2).png
LV_IMG_DECLARE( ui_img_logo_2_png);   // assets/logo_2.png


extern int16_t minutes;
extern int16_t seconds; 
// extern char minText[10];
// extern char secText[10];

//Current temperature
extern int16_t GrillTemperatureC;
extern int16_t WarmerTemperatureC;
//In farenheit
extern int16_t GrillTemperatureF;
extern int16_t WarmerTemperatureF;

extern bool Fahrenheit ;
extern bool Celcius;


extern int16_t GrillSetTemperatureC;
extern int16_t WarmerSetTemperatureC;

extern int16_t GrillSetTemperatureF;
extern int16_t WarmerSetTemperatureF;





void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
