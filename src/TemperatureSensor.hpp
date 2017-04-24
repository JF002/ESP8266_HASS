#include <SimpleDHT.h>

class TemperatureSensor {
  public:
    void Loop() {
      sensor.read(pin, &temperature, &humidity, NULL);
    }

    byte Temperature() { return temperature; }
    byte Humidity() { return humidity; }

  private:
    SimpleDHT11 sensor;
    byte temperature;
    byte humidity;
    const int pin = D4;
};
