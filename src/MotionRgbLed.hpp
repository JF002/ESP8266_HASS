#pragma once
#include <Arduino.h>
#include <NeoPixelBus.h>

//NeoPixelBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod> strip(1, 4); // 4 = D2

class MotionRgbLed {
  public:
     MotionRgbLed(NeoPixelBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod>* strip) {
       this->strip = strip;
     }

     void Setup() {
       // this resets all the neopixels to an off state
       strip->Begin();
       strip->Show();

       direction = -1;
       brightness = 0.0;

       strip->SetPixelColor(0, HslColor(0.08, 1.0, 0));
     }

     void Loop() {
       switch(direction) {
         case 1:
            if(brightness < max) {
              brightness += direction * (0.001);
              auto color = HslColor(0.08, 1.0, brightness);
              strip->SetPixelColor(0, color);
              strip->Show();
            }
            break;
          case -1:
            if(brightness > min) {
              brightness += direction * (0.001);
              auto color = HslColor(0.08, 1.0, brightness);
              strip->SetPixelColor(0, color);
              strip->Show();
            }
            break;
       }
     }

     void Set() {
       direction = 1;
     }

     void Clear() {
       direction = -1;
     }

  private:
    NeoPixelBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod>* strip;
    int direction;
    float brightness;
    const float max = 0.4;
    const float min = 0.001;
};
