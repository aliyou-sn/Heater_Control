#ifndef _UI_EVENTS_H
#define _UI_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif


//Relay Pins
#define Grill 14
#define Warmer 12

#define TFT_BL_PIN 32      // Pin 32 for backlight control
#define PWM_CHANNEL 0      // PWM channel
#define PWM_FREQUENCY 5000 // PWM frequency (5kHz)
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

void Grill_On(lv_event_t * e);
void Grill_Off(lv_event_t * e);
void Warmer_On(lv_event_t * e);
void Warmer_Off(lv_event_t * e);
void DecrementGTemperature(lv_event_t * e);
void DecrementWTemperature(lv_event_t * e);
void IncrementGTemperature(lv_event_t * e);
void IncrementWTemperature(lv_event_t * e);
void SwapCelsiusFarenheit(lv_event_t * e);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
