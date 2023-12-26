#include <Arduino.h>
#include <FastLED.h>
//#include <SPI.h>
#define FASTLED_INTERNAL
#define PIN_HALLSENSOR 1
#define PIN_BUTTON     0
#define PIN_LED        21
#define NUM_LEDS       1


CRGB leds[NUM_LEDS];
uint8_t led_ih             = 0;
uint8_t led_status         = 0;
String led_status_string[] = {"Rainbow", "Red", "Green", "Blue"};

uint64_t timeStamp = 0;
uint64_t ledTimeStamp = 0;
bool hallSensorState = false;

void setup() {
  USBSerial.begin(115200);  
  pinMode(PIN_HALLSENSOR, INPUT);
  pinMode(PIN_BUTTON, INPUT);
  FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);
}

int count = 0;

void loop() {
  if (millis() - timeStamp > 100)
  {
    timeStamp = millis();
    hallSensorState = digitalRead(PIN_HALLSENSOR);
    USBSerial.printf("VALUE : %d\r\n", hallSensorState);
    
  }

  if(hallSensorState != digitalRead(PIN_HALLSENSOR))
  {
    USBSerial.println("State Changed");
  }

  if(millis() - ledTimeStamp > 15)
  {
    switch (led_status) {
        case 0:
            leds[0] = CHSV(led_ih, 255, 255);
            break;
        case 1:
            leds[0] = CRGB::Red;
            break;
        case 2:
            leds[0] = CRGB::Green;
            break;
        case 3:
            leds[0] = CRGB::Blue;
            break;
        default:
            break;
    }
    FastLED.show();
    led_ih++;
  }


  if(!digitalRead(PIN_BUTTON))
  {
    delay(5);
    if (!digitalRead(PIN_BUTTON)) 
    {
      led_status++;
      if (led_status > 3) led_status = 0;
      while (!digitalRead(PIN_BUTTON))
          ;
      USBSerial.print("LED status updated: ");
      USBSerial.println(led_status_string[led_status]);
    }
  }
}

