#include <Arduino.h>
#include "M5Atom.h"
#include "neopixel.h"
#include <SPI.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <IRsend.h>

#define IR_INTERVAL 10

long count = 0;
MyNeopixel* myNeopixel = new MyNeopixel();

#ifdef ARDUINO_ESP32C3_DEV
const uint16_t kRecvPin = 10;  // 14 on a ESP32-C3 causes a boot loop.
#else  
const uint16_t kRecvPin = 25;
const uint16_t kIrLed   = 26;
#endif  


IRrecv irrecv(kRecvPin);
IRsend irsend(kIrLed);
decode_results results;

uint64_t lastTime = 0;


// 8byte unsinged int
// 20DF10EF
// E0E040BF
// 122430CF

uint32_t dataArray[3] = {0,};
uint32_t startButton[3] = {0x20DF10EF, 0xE0E040BF, 0x122430CF };


void setup() {
  M5.begin(true, true, false);
  myNeopixel->InitNeopixel();
  irrecv.enableIRIn();
  irsend.begin();
  delay(50);

  Serial.println("- Start Atom -");
  myNeopixel->pickOneLED(0, myNeopixel->strip->Color(255, 0, 100), 50, 1);

  
}

int receiveCount = 0;
void loop() {
  if(M5.Btn.wasPressed())
  {
    Serial.printf("Count : %d\r\n", count++);
    // for (int i = 0; i < sizeof(startButton)/sizeof(uint32_t); i++)
    // {
    //   irsend.sendNEC(startButton[i]);
    //   Serial.printf("0x%x\r\n", startButton[i]);
    //   delay(50);
    // }

    delay(50);
    // initialize
    for (size_t i = 0; i < sizeof(dataArray)/sizeof(uint32_t); i++)
    {
      Serial.printf("Return Value : 0x%x\r\n", dataArray[i]);
      if(dataArray[i] != 0)
      {
        irsend.sendNEC(dataArray[i]);
      }
      dataArray[i] = 0;
    }
    receiveCount = 0;
    myNeopixel->pickOneLED(0, myNeopixel->strip->Color(255, 0, 0), 50, 1);
  }
  
  if(millis() - lastTime > IR_INTERVAL)
  {
    lastTime = millis();
    if (irrecv.decode(&results)) 
    {
      // print() & println() can't handle printing long longs. (uint64_t)
      if ((results.value & 0xFFFFFFFFFFFFFFFF) != 0xFFFFFFFFFFFFFFFF)
      {
        if(receiveCount >= sizeof(startButton)/sizeof(uint32_t))
        {
          for (size_t i = 0; i < sizeof(dataArray)/sizeof(uint32_t); i++)
          {
            dataArray[i] = 0;
          }
        }
        else
        {
          dataArray[receiveCount++] = results.value;
          myNeopixel->pickOneLED(0, myNeopixel->strip->Color(0, 255, 0), 50, 1);
        }

        
        // if(results.value & 0xE0E040BF)
        // {
        //   myNeopixel->pickOneLED(0, myNeopixel->strip->Color(0, 255, 0), 50, 1);
        // }
        serialPrintUint64(results.value, HEX);
        Serial.println("");
        irrecv.resume();  // Receive the next value
      }
            
    }
  }

  M5.update();  // M5.Btn.read();

  if(lastTime > 4000000000)
  {
    ESP.restart();
  }
}



