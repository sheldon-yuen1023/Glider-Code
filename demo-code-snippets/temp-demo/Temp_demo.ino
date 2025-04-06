#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 10  // GPIO10 = D10 on XIAO ESP32-C3

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);
  sensors.begin();
  delay(1000);
  Serial.println("DS18B20 Temperature Monitor");
}

void loop() {
  sensors.requestTemperatures();  // Ask all sensors to update temperature
  int count = sensors.getDeviceCount();
  Serial.print("Total sensors detected: ");
  Serial.println(count);

  for (int i = 0; i < count; i++) {
    float tempC = sensors.getTempCByIndex(i);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    if (tempC == DEVICE_DISCONNECTED_C) {
      Serial.println("Disconnected or error!");
    } else {
      Serial.print(tempC);
      Serial.println(" °C");
    }
  }

  Serial.println("-----------------------------");
  delay(2000);
}
