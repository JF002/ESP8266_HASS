#include "JsonRgbLed.hpp"
#include <ArduinoJson.h>
#include <utility>

void JsonRgbLed::Setup() {
  strip->Begin();
  isModified = true;
}

void JsonRgbLed::Apply(const JsonObject& root) {
  if(root.containsKey("state")) {
    if(root["state"] == "ON") {
      if(state == false) {
        current = RgbColor(0,0,0);
        target = currentColor;
      }

      state = true;

      if(root.containsKey("brightness")) {
        brightness = root["brightness"];
        current = currentColor;
        target = targetColor;
      }

      if(root.containsKey("effect")) {
        //this->effect = root["effect"].as<String>();
      }

      if (root.containsKey("color")) {
        JsonObject& array = root["color"];
        uint8_t r = array["r"];
        uint8_t g = array["g"];
        uint8_t b = array["b"];

        current = currentColor;
        target = RgbColor(r, g, b);
      }
    }
    else {
      state = false;
      current = currentColor;
      target = RgbColor(0,0,0);
    }
    ApplyRgb();
  }
}

void JsonRgbLed::ToJson(JsonObject& rootState) const {
  bool state = this->State();
  String output;
  rootState["state"] = state?"ON" : "OFF";

  auto& colorArray = rootState.createNestedObject("color");
  colorArray["r"] = this->R();
  colorArray["g"] = this->G();
  colorArray["b"] = this->B();
  rootState["brightness"] = this->Brightness();
}

bool JsonRgbLed::State() const {
  return this->state;
}

void JsonRgbLed::ApplyRgb() {
  AnimUpdateCallback animUpdate = [this](const AnimationParam& param) {
    float progress = NeoEase::QuadraticInOut(param.progress);
    RgbColor updatedColor = RgbColor::LinearBlend(current, target, progress);
    strip->SetBrightness(brightness);
    strip->SetPixelColor(0, updatedColor);
    if(param.state == AnimationState::AnimationState_Completed) {
      isModified = true;
      if(state) {
        currentColor = target;
        targetColor = target;
      }
    }
  };
  animations->StartAnimation(0, 100, animUpdate);
}

bool JsonRgbLed::Loop() {
  bool ret = isModified;
  isModified = false; // ATOMIC!!!

  if(firstRun) {
    firstRun = false;
    strip->SetPixelColor(0,RgbColor(0,0,0));
    strip->Show();
  }

  if (animations->IsAnimating()) {
    animations->UpdateAnimations();
    strip->Show();
  }

  return ret;
}
