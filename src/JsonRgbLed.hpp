#pragma once

#include <NeoPixelBrightnessBus.h>
#include <NeoPixelAnimator.h>
#include <ArduinoJson.h>
#include <atomic>

class JsonRgbLed {
  public:
    JsonRgbLed(NeoPixelBrightnessBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod> *strip) {
      this->strip = strip;
      state = false;
      brightness = 255;
      currentColor = RgbColor(255,255,255);
      isModified = true;
      firstRun = true;
      animations = new NeoPixelAnimator(1, NEO_CENTISECONDS);
    }

    ~JsonRgbLed() {
      delete(animations);
    }

    void Apply(const JsonObject& root);
    bool State() const;
    uint8_t R() { return currentColor.R;}
    uint8_t G() { return currentColor.G;}
    uint8_t B() { return currentColor.B; }
    uint8_t Brightness() { return brightness; }

    void Setup();
    bool Loop();

  private:
    NeoPixelBrightnessBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod> *strip;
    NeoPixelAnimator* animations;
    bool state;
    uint8_t brightness;
    String effect;
    RgbColor currentColor;
    RgbColor targetColor;

    RgbColor current, target;

    bool isModified;
    bool firstRun;


    void ApplyBrightness();
    void ApplyEffect(){
      Serial.println("Effect : " + effect);
    }
    void ApplyRgb();
};
