#include <Arduino.h>
#include <DHT_U.h>
#include "DHT.h"
#include "Adafruit_Sensor.h"
#include <BluetoothSerial.h>

#define DHTPIN 14
#define DHTTYPE    DHT11


DHT_Unified dht(DHTPIN, DHTTYPE);
BluetoothSerial ESP_BT;
const int LDR = 33;

uint32_t delayMS;
bool lightsOn = false;
void setup() {
  Serial.begin(115200);
  ESP_BT.begin("ESP32Telem");
  Serial.println("Bluetooth baslatildi");
  pinMode(32,OUTPUT);
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  delay(delayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    ESP_BT.printf("TEMP=%6.2f /",event.temperature);
    Serial.println(F("C"));
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    ESP_BT.printf("HUMD=%6.2f /",event.relative_humidity);
  }

  int LDRVal = analogRead(LDR);
  Serial.println(LDRVal);

  ESP_BT.printf("LDRL=%d /",LDRVal);

  if (ESP_BT.available()) {
    //48= 0 49=1 ascii
    int lightData = ESP_BT.read();
    Serial.print("Gelen veri: ");
    Serial.println(lightData);
    if (lightData == 48) {
      digitalWrite(32,LOW);
    }else if(lightData == 49) {
      digitalWrite(32,HIGH);
    }
  }

}
