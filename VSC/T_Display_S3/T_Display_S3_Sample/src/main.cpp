#include <Arduino.h>

#include "OneButton.h"
#include "Wire.h"




// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
// #error  "The current version is not supported for the time being, please use a version below Arduino ESP32 3.0"
// #endif

// #warning Please confirm that you have purchased a display screen with a touch chip, otherwise the touch routine cannot be implemented.
// #if defined(TOUCH_MODULES_CST_MUTUAL)
// TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, CTS328_SLAVE_ADDRESS, PIN_TOUCH_RES);
// #elif defined(TOUCH_MODULES_CST_SELF)
// TouchLib touch(Wire, PIN_IIC_SDA, PIN_IIC_SCL, CTS820_SLAVE_ADDRESS, PIN_TOUCH_RES);
// #else
// #error "Please choose the correct touch driver model!"
// #endif

#define TOUCH_GET_FORM_INT 0


void setup()
{

    Serial.begin(115200);
    Serial.println("Hello T-Display-S3");



}

void loop()
{
    
}

