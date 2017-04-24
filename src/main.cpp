#include <ESP8266wifi.h>
#include <PubSubClient.h>

#include "MotionSensor.hpp"
#include "MotionRgbLed.hpp"
#include "TemperatureSensor.hpp"
#include "JsonRgbLed.hpp"
#include <ArduinoJson.h>

void printWifiStatus();

WiFiClient espClient;
PubSubClient client(espClient);
int cpt;
MotionSensor motion;

TemperatureSensor temperature;

NeoPixelBrightnessBus<::NeoGrbFeature, ::NeoEsp8266BitBang800KbpsMethod> strip(1, 4); // 4 = D2
//MotionRgbLed motionRgbled(&strip);

JsonRgbLed jsonRgbLed(&strip);



void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char buffer[length+1];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';
  Serial.println("Message in " + String(topic) + " -- " + String(length));
  if(String(topic) == String("/motion")) {
    Serial.println(String(length));
    /*if(length == 2 && payload[0] == 'O' && payload[1] == 'N')
      motionRgbled.Set();
    else if(length == 3 && payload[0] == 'O' && payload[1] == 'F' && payload[2] == 'F')
      motionRgbled.Clear();
      */
  }
  else if (String(topic) == String("/rgbled/command")) {
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

  Serial.println("Connecting to MQTT borker");
  client.setServer("192.168.1.109", 1883);
  client.connect("WemosD1");
  client.setCallback(mqttCallback);
  client.subscribe("/motion");
  client.subscribe("/rgbled/command");

  //motionRgbled.Setup();
    jsonRgbLed.Setup();
  Serial.println("Setup finished");
  cpt = 0;
}

void loop() {
  client.loop();
  //motionRgbled.Loop();
  if(jsonRgbLed.Loop()) {
    StaticJsonBuffer<200> jsonBuffer2;
    bool state = jsonRgbLed.State();
    String output;
    JsonObject& rootState = jsonBuffer2.createObject();
    rootState["state"] = state?"ON" : "OFF";

    auto& colorArray = rootState.createNestedObject("color");
    colorArray["r"] = jsonRgbLed.R();
    colorArray["g"] = jsonRgbLed.G();
    colorArray["b"] = jsonRgbLed.B();
    rootState["brightness"] = jsonRgbLed.Brightness();
    rootState.printTo(output);
    client.publish("/rgbled/state", output.c_str());
  }


  motion.Loop();
  if(motion.IsMotionStateChanged()) {
    client.publish("/motion", motion.IsMotion() ? String("ON").c_str() : String("OFF").c_str());
  }

  if(cpt % 6000 == 0) {
    temperature.Loop();
    client.publish("/temperature", String(temperature.Temperature()).c_str());
    client.publish("/humidity", String(temperature.Humidity()).c_str());
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
