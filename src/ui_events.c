#include <Arduino.h>
#include "ui.h"

int16_t GrillSetTemperatureC = 200;
int16_t WarmerSetTemperatureC = 85;

int16_t GrillSetTemperatureF = 392;
int16_t WarmerSetTemperatureF = 185;


bool Fahrenheit = false;
bool Celcius = true;

void Grill_On(lv_event_t * e)
{
	digitalWrite(Grill,HIGH);
}

void Grill_Off(lv_event_t * e)
{
	digitalWrite(Grill,LOW);
}

void Warmer_On(lv_event_t * e)
{
	digitalWrite(Warmer,HIGH);
}

void Warmer_Off(lv_event_t * e)
{
    digitalWrite(Warmer,LOW);
}

void DecrementGTemperature(lv_event_t * e)
{

    char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
	if(Celcius)
    {
        if (GrillSetTemperatureC > 100 && GrillSetTemperatureC <= 270 ) 
        {
        GrillSetTemperatureC -= 10;
        lv_snprintf(buf, sizeof(buf), "%d", GrillSetTemperatureC);
        lv_label_set_text(ui_GSetTempLabel, buf);
        }
    }
    else
    {
        if (GrillSetTemperatureF > 212 && GrillSetTemperatureF <= 512 ) 
        {
        GrillSetTemperatureF -= 10;
        lv_snprintf(buf, sizeof(buf), "%d", GrillSetTemperatureF);
        lv_label_set_text(ui_GSetTempLabel,buf);
        }

    }
}

void DecrementWTemperature(lv_event_t * e)
{
   char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
   if(Celcius)
   {
       if (WarmerSetTemperatureC > 50 && WarmerSetTemperatureC <=130 ) {
       WarmerSetTemperatureC -= 5;
       lv_snprintf(buf, sizeof(buf), "%d", WarmerSetTemperatureC);
       lv_label_set_text(ui_WSetTempLabel,buf);
       }
   }
   else
   {

    if (WarmerSetTemperatureF > 122 && WarmerSetTemperatureF <= 215 ) {
       WarmerSetTemperatureF -= 5;
       lv_snprintf(buf, sizeof(buf), "%d", WarmerSetTemperatureF);
       lv_label_set_text(ui_WSetTempLabel,buf);
       }

   }
}

void IncrementGTemperature(lv_event_t * e)
{
   char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
   if(Celcius)
    {
        if (GrillSetTemperatureC >= 90 && GrillSetTemperatureC < 260 ) 
        {
        GrillSetTemperatureC += 10;
        lv_snprintf(buf, sizeof(buf), "%d", GrillSetTemperatureC);
        lv_label_set_text(ui_GSetTempLabel,buf);
        }
    }
    else
    {
        if (GrillSetTemperatureF >= 212 && GrillSetTemperatureF < 500 ) 
        {
        GrillSetTemperatureF += 10;
        lv_snprintf(buf, sizeof(buf), "%d", GrillSetTemperatureF);
        lv_label_set_text(ui_GSetTempLabel,buf);
        }

    }
}

void IncrementWTemperature(lv_event_t * e)
{
    char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
    if(Celcius)
   {
       if (WarmerSetTemperatureC >= 45 && WarmerSetTemperatureC < 120 ) {
       WarmerSetTemperatureC += 5;
       lv_snprintf(buf, sizeof(buf), "%d", WarmerSetTemperatureC);
       lv_label_set_text(ui_WSetTempLabel,buf);
       }
   }
   else
   {

    if (WarmerSetTemperatureF >= 115 && WarmerSetTemperatureF <=212 ) {
       WarmerSetTemperatureF += 5;
       lv_snprintf(buf, sizeof(buf), "%d", WarmerSetTemperatureF);
       lv_label_set_text(ui_WSetTempLabel,buf);
       }

   }
}

void SwapCelsiusFarenheit(lv_event_t * e)
{
    if(Fahrenheit == true && Celcius == false)
    {
        lv_label_set_text(ui_CelsiusFareinheitLabel,"°C");
        lv_label_set_text(ui_CelsiusFarenheitLabel,"°C");
        lv_label_set_text(ui_GSetTempLabel,"200");
        lv_label_set_text(ui_WSetTempLabel,"85");
        GrillSetTemperatureC = 200;
        WarmerSetTemperatureC = 85;

        Fahrenheit = false;
        Celcius = true;
    }
    else
    {
        lv_label_set_text(ui_CelsiusFareinheitLabel,"°F");
        lv_label_set_text(ui_CelsiusFarenheitLabel,"°F");
        lv_label_set_text(ui_GSetTempLabel,"392");
        lv_label_set_text(ui_WSetTempLabel,"185");

        GrillSetTemperatureF = 392;
        WarmerSetTemperatureF = 185;

        Fahrenheit = true;
        Celcius = false;
    }
}

