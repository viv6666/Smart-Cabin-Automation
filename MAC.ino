#include <WiFi.h>  // << This is required

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // Set Wi-Fi to Station mode
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // nothing here
}
