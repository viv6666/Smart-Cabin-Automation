#include <esp_flash.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Erasing flash (NVS)...");
  for (int i = 0; i < 100; i++) {
    esp_flash_erase_region(nullptr, i * 4096, 4096);
  }
  Serial.println("Flash erase complete. Now re-upload main code.");
}

void loop() {}
