#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"
#include <time.h>
#include <max6675.h>     

// Actual Temperature for Grill and warmer
//In celcius
int16_t GrillTemperatureC;
int16_t WarmerTemperatureC;
//In farenheit
int16_t GrillTemperatureF;
int16_t WarmerTemperatureF;


int thermoDO = 19;
int thermoCLK = 18;
int thermoCS_T1 = 26;
int thermoCS_T2 = 25;

int triac1Pin = 17;
int triac2Pin = 22;

int zeroCrossPin = 5; // Zero-crossing detection pin
volatile bool zeroCrossDetected = false;
volatile unsigned long lastZeroCrossTime = 0;


MAX6675 thermocouple_1(thermoCLK, thermoCS_T1, thermoDO);
MAX6675 thermocouple_2(thermoCLK, thermoCS_T2, thermoDO);


/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

// TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */
TFT_eSPI tft = TFT_eSPI();

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX, touchY;

    bool touched = tft.getTouch( &touchX, &touchY, 600 );

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
    }
}


// Task to update system time on the display

void displayTimeTask(void *Parameters) {    
    for (;;) 
    {

        // Get current system time
        time_t now = time(NULL);
        struct tm *local = localtime(&now);
        
        // Format time as "MM:SS"
        char minutesStr[3];
        char secondsStr[3];
        lv_snprintf(minutesStr, sizeof(minutesStr), "%02d", local->tm_hour);
        lv_snprintf(secondsStr, sizeof(secondsStr), "%02d", local->tm_min);


        // Update label text
        lv_label_set_text(ui_MinutesLabel, minutesStr);
        lv_label_set_text(ui_SecondsLabel, secondsStr);        
    
        // Delay for 30 second
        vTaskDelay(30000 / portTICK_PERIOD_MS);

    }
}


void adjustTriac(int triacPin, float currentTemp, float setpointTemp) {
  float tempDiff = setpointTemp - currentTemp;
  int firingDelay = map(tempDiff, 0, 10, 0, 8000); 

  delayMicroseconds(firingDelay); 
  digitalWrite(triacPin, HIGH);
  delayMicroseconds(10);  
  digitalWrite(triacPin, LOW);
}


//Function to get current temperature of grill and warmer every 1 sec
void GetTemp(void *parameter){
  for(;;)
  {

  GrillTemperatureC = thermocouple_1.readCelsius();
  WarmerTemperatureC = thermocouple_2.readCelsius();
  GrillTemperatureF = thermocouple_1.readFahrenheit();
  WarmerTemperatureF = thermocouple_2.readFahrenheit();

  char buf1[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  char buf2[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  if(Fahrenheit)
  {

    if(digitalRead(Grill) == HIGH && GrillTemperatureF < GrillSetTemperatureF - 5)
    {
      if (zeroCrossDetected) 
      {
        adjustTriac(triac1Pin, GrillTemperatureF, GrillSetTemperatureF);
      }
    }
    else if(GrillTemperatureF >= GrillSetTemperatureF)
    {
      digitalWrite(Grill, LOW);
      _ui_state_modify( ui_GrillSwitch, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      lv_label_set_text(ui_Label3, "OFF");
    }


    if(digitalRead(Warmer) == HIGH && WarmerTemperatureF < WarmerSetTemperatureF - 5)
    {
      if (zeroCrossDetected) 
      {
        adjustTriac(triac1Pin, WarmerTemperatureF, WarmerSetTemperatureF);
      }
    }
    else if(WarmerTemperatureF >= WarmerSetTemperatureF)
    {
      digitalWrite(Warmer, LOW);
      _ui_state_modify( ui_Switch2, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      lv_label_set_text(ui_Label4, "OFF");
    }
    

    lv_snprintf(buf1, sizeof(buf1), "%d", GrillTemperatureF);
    lv_label_set_text(ui_GCurrentTempLabel, buf1);
    lv_snprintf(buf2, sizeof(buf2), "%d", WarmerTemperatureF);
    lv_label_set_text(ui_WCurrentTempLabel, buf2);

     Serial.println("In Fahrenheit");
  }
  else
  {


    if(digitalRead(Grill) == HIGH && GrillTemperatureC < GrillSetTemperatureC - 5)
    {
      if (zeroCrossDetected) 
      {
        adjustTriac(triac1Pin, GrillTemperatureC, GrillSetTemperatureC);
      }
    }
    else if(GrillTemperatureC >= GrillSetTemperatureC)
    {
      digitalWrite(Grill, LOW);
      _ui_state_modify( ui_GrillSwitch, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      lv_label_set_text(ui_Label3, "OFF");
    }


    if(digitalRead(Warmer) == HIGH && WarmerTemperatureC < WarmerSetTemperatureC - 5)
    {
      if (zeroCrossDetected) 
      {
        adjustTriac(triac1Pin, WarmerTemperatureC, WarmerSetTemperatureC);
      }
    }
    else if(WarmerTemperatureC >= WarmerSetTemperatureC)
    {
      digitalWrite(Warmer, LOW);
      _ui_state_modify( ui_Switch2, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      lv_label_set_text(ui_Label4, "OFF");
    }
    


    lv_snprintf(buf1, sizeof(buf1), "%d", GrillTemperatureC);
    lv_label_set_text(ui_GCurrentTempLabel, buf1);
    lv_snprintf(buf2, sizeof(buf2), "%d", WarmerTemperatureC);
    lv_label_set_text(ui_WCurrentTempLabel, buf2);

    Serial.println("In Celcius");
   
    }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void IRAM_ATTR zeroCrossISR() {
  zeroCrossDetected = true;
  lastZeroCrossTime = micros();
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */


    lv_init();


    // Set up PWM for backlight on pin 32
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(TFT_BL_PIN, PWM_CHANNEL);

    // Set initial brightness (80%)
    ledcWrite(PWM_CHANNEL, 205);

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( 1 ); /* Landscape orientation, flipped */

    // Calibration Values for my Setup
    uint16_t calData[5] = { 247, 3669, 293, 3407, 7 };
    tft.setTouch(calData);

    // lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );
    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    ui_init();

    pinMode(Grill, OUTPUT);
    pinMode(Warmer, OUTPUT);
    pinMode(zeroCrossPin, INPUT_PULLUP);
    pinMode(triac1Pin, OUTPUT);
    pinMode(triac2Pin, OUTPUT);

    // Attach zero-crossing interrupt
    attachInterrupt(digitalPinToInterrupt(zeroCrossPin), zeroCrossISR, FALLING);


    //Current temperature task
    xTaskCreate(GetTemp,"Update temperature", 5000, NULL, 2, NULL);

    //Current temperature task
    xTaskCreate(displayTimeTask,"display Time", 5000, NULL, 2, NULL);

    Serial.println( "Setup done" );

    digitalWrite(Grill, LOW);
    digitalWrite(Warmer, LOW);
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    // delay(5);
}
