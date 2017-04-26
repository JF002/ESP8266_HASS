#include <ESP8266wifi.h>
#include <PubSubClient.h>

#include "MotionSensor.hpp"
#include "MotionRgbLed.hpp"
#include "TemperatureSensor.hpp"
#include "JsonRgbLed.hpp"
#include <ArduinoJson.h>

extern "C" {
#include "user_interface.h"
}


void printWifiStatus();
void GetUniqueId();
void BuildTopicNames();

String uniqueId;
WiFiClient espClient;
PubSubClient client(espClient);
int cpt;
MotionSensor motion;

TemperatureSensor temperature;
NeoPixelBrightnessBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod> strip(1, 4); // 4 = D2
//MotionRgbLed motionRgbled(&strip);

JsonRgbLed jsonRgbLed(&strip);

String topic_temperature;
String topic_humidity;
String topic_motion;
String topic_rgbLedState;
String topic_rgbLedCommand;
String topic_diag_freeMemory;
String topic_diag_uptime;
uint32_t uptime = 0;


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char buffer[length+1];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';

  if (String(topic) == String(topic_rgbLedCommand.c_str())) {
    StaticJsonBuffer<200> jsonBuffer;
    Serial.println(String((char*)buffer).c_str());
    JsonObject& root = jsonBuffer.parseObject(buffer);
    if (root.success()) {
      jsonRgbLed.Apply(root);
    }
  }
}

void setup() {
  Serial.begin(921600);

  Serial.println("Hello!");
  Serial.println("Configuring MotionSensor");
  motion.Setup();
  Serial.println("Connecting to Wifi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin("ABCD", "1234");
  while(WiFi. status() != WL_CONNECTED) {
    int status = WiFi.status();
    Serial.println("Wifi status : " + String(status));
    delay(1000);
  }

  printWifiStatus();
  GetUniqueId();
  BuildTopicNames();

  Serial.println("Unique Id : " + uniqueId);

  Serial.println("Topic temperature : " + topic_temperature);
  Serial.println("Topic humidity : " + topic_humidity);
  Serial.println("Topic motion : " + topic_motion);
  Serial.println("Topic RGBLED state : " + topic_rgbLedState);
  Serial.println("Topic RGBLED command : " + topic_rgbLedCommand);
  Serial.println("Topic Diag - Free Memory : " +topic_diag_freeMemory);
  Serial.println("Topic Diag - Uptime : " + topic_diag_uptime);


  Serial.println("Connecting to MQTT borker");
  client.setServer("192.168.1.109", 1883);
  client.connect("WemosD1");
  client.setCallback(mqttCallback);
  client.subscribe(topic_rgbLedCommand.c_str());

  jsonRgbLed.Setup();
  Serial.println("Setup finished");
  cpt = 0;
}

void RgbLedLoop() {
  if(jsonRgbLed.Loop()) {
    StaticJsonBuffer<200> jsonBuffer2;
    JsonObject& rootState = jsonBuffer2.createObject();
    String output;
    jsonRgbLed.ToJson(rootState);
    rootState.printTo(output);
    Serial.println(output);
    client.publish(topic_rgbLedState.c_str(), output.c_str());
  }
}

void MotionLoop() {
  motion.Loop();
  if(motion.IsMotionStateChanged()) {
    client.publish(topic_motion.c_str(), motion.IsMotion() ? String("ON").c_str() : String("OFF").c_str());
  }
}

void TemperatureLoop() {
  temperature.Loop();
  client.publish(topic_temperature.c_str(), String(temperature.Temperature()).c_str());
  client.publish(topic_humidity.c_str(), String(temperature.Humidity()).c_str());
}

void DiagnosticLoop() {
  uptime++;
  client.publish(topic_diag_freeMemory.c_str(), String(system_get_free_heap_size()).c_str());
  client.publish(topic_diag_uptime.c_str(), String(uptime).c_str());
}

void loop() {
  client.loop();

  RgbLedLoop();
  MotionLoop();

  if(cpt % 6000 == 0) {
    TemperatureLoop();
    DiagnosticLoop();
  }

  cpt++;

  delay(10);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void GetUniqueId() {
  byte mac[6];
  WiFi.macAddress(mac);

  uniqueId = "";
  uniqueId += String(mac[5], HEX) + ":" +
              String(mac[4], HEX) + ":" +
              String(mac[3], HEX) + ":" +
              String(mac[2], HEX) + ":" +
              String(mac[1], HEX) + ":" +
              String(mac[0], HEX);
}

void BuildTopicNames() {
  topic_temperature = "/" + uniqueId + "/temperature";
  topic_humidity = "/" + uniqueId + "/humidity";
  topic_motion = "/" + uniqueId + "/motion";
  topic_rgbLedState = "/" + uniqueId + "/rgbled/state";
  topic_rgbLedCommand = "/" + uniqueId + "/rgbled/command";

  topic_diag_freeMemory = "/" + uniqueId + "/diag/freeMemory";
  topic_diag_uptime = "/" + uniqueId + "/diag/uptime";
}
