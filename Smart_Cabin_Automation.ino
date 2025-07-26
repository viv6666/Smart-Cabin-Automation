//Smart Cabin Automation system by Kamanuru Venkata Sai ViveK
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <AceButton.h>
using namespace ace_button;

// BLE Provisioning Info
const char* service_name = "CABIN";
const char* pop = "ece1234";

// Pin Mapping
#define SWITCH1    32   // Light 1 wall switch
#define SWITCH2    35   // Light 2 wall switch
#define SWITCH3    34   // Light 3 wall switch
#define SWITCH4    39   // Light 4 wall switch

#define FAN_SWITCH 33   // Fan ON/OFF wall switch

#define RELAY1     15   // Light 1 relay
#define RELAY2     2    // Light 2 relay
#define RELAY3     4    // Light 3 relay
#define RELAY4     22   // Light 4 relay
#define FAN_RELAY  18   // Fan max speed relay (single relay)

// Reset button
#define GPIO_RESET 0    // Onboard reset button

// State variables
bool light_state[4] = {false, false, false, false};
bool fan_is_on = false;

// RainMaker Devices
Switch my_switch1("Light1");
Switch my_switch2("Light2");
Switch my_switch3("Light3");
Switch my_switch4("Light4");
Switch my_fan("Fan");

// Buttons
ButtonConfig config1, config2, config3, config4, config_fan;
AceButton button1(&config1), button2(&config2), button3(&config3), button4(&config4), fan_button(&config_fan);

void fan_set(bool on) {
  digitalWrite(FAN_RELAY, on ? HIGH : LOW);
  fan_is_on = on;
}

// RainMaker write callback
void write_callback(Device *device, Param *param, const param_val_t val, void *, write_ctx_t *) {
  const char* dev = device->getDeviceName();
  const char* pname = param->getParamName();

  if (strcmp(dev, "Fan") == 0) {
    if (strcmp(pname, "Power") == 0) {
      bool on = val.val.b;
      fan_set(on);
      my_fan.updateAndReportParam("Power", on);
    }
  } else {
    // Lights
    bool state = val.val.b;
    if      (dev && strcmp(dev, "Light1") == 0) { light_state[0] = state; digitalWrite(RELAY1, state); }
    else if (dev && strcmp(dev, "Light2") == 0) { light_state[1] = state; digitalWrite(RELAY2, state); }
    else if (dev && strcmp(dev, "Light3") == 0) { light_state[2] = state; digitalWrite(RELAY3, state); }
    else if (dev && strcmp(dev, "Light4") == 0) { light_state[3] = state; digitalWrite(RELAY4, state); }
    param->updateAndReport(val);
  }
}

// Button handlers
void handle_button(AceButton* btn, uint8_t eventType, uint8_t) {
  if (eventType != AceButton::kEventPressed) return;

  int index = -1;
  if      (btn == &button1) index = 0;
  else if (btn == &button2) index = 1;
  else if (btn == &button3) index = 2;
  else if (btn == &button4) index = 3;

  if (index >= 0) {
    light_state[index] = !light_state[index];
    int relay_gpio = (index == 0) ? RELAY1 :
                     (index == 1) ? RELAY2 :
                     (index == 2) ? RELAY3 : RELAY4;
    digitalWrite(relay_gpio, light_state[index]);
    Switch* d = (index == 0) ? &my_switch1 :
                (index == 1) ? &my_switch2 :
                (index == 2) ? &my_switch3 : &my_switch4;
    d->updateAndReportParam("Power", light_state[index]);
  }
}

void handle_fan_button(AceButton* btn, uint8_t eventType, uint8_t) {
  if (eventType != AceButton::kEventPressed) return;
  fan_set(!fan_is_on);
  my_fan.updateAndReportParam("Power", fan_is_on);
}

// Setup
void setup() {
  Serial.begin(115200);

  // Initialize outputs (relays OFF)
  int outs[] = {RELAY1, RELAY2, RELAY3, RELAY4, FAN_RELAY};
  for (int i = 0; i < 5; ++i) {
    pinMode(outs[i], OUTPUT);
    digitalWrite(outs[i], LOW);
  }

  // Inputs (switches, reset)
  int ins[] = {SWITCH1, SWITCH2, SWITCH3, SWITCH4, FAN_SWITCH, GPIO_RESET};
  for (int i = 0; i < 6; ++i) {
    pinMode(ins[i], INPUT_PULLUP);
  }

  // Setup button debounces
  config1.setDebounceDelay(50);
  config2.setDebounceDelay(50);
  config3.setDebounceDelay(50);
  config4.setDebounceDelay(50);
  config_fan.setDebounceDelay(50);

  // Initialize buttons with handlers
  config1.setEventHandler(handle_button); button1.init(SWITCH1);
  config2.setEventHandler(handle_button); button2.init(SWITCH2);
  config3.setEventHandler(handle_button); button3.init(SWITCH3);
  config4.setEventHandler(handle_button); button4.init(SWITCH4);
  config_fan.setEventHandler(handle_fan_button); fan_button.init(FAN_SWITCH);

  // RainMaker devices and callbacks
  Node my_node = RMaker.initNode("Room Controller");
  my_fan.addCb(write_callback);
  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback);
  my_switch4.addCb(write_callback);

  my_node.addDevice(my_fan);
  my_node.addDevice(my_switch1);
  my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);
  my_node.addDevice(my_switch4);

  // OTA, timezone, RainMaker start
  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.start();

  // BLE provisioning QR print
  WiFi.onEvent([](arduino_event_t *sys_event) {
    if (sys_event->event_id == ARDUINO_EVENT_PROV_START) {
      printQR(service_name, pop, "ble");
      Serial.println("BLE provisioning started...");
    }
  });

  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,
                          WIFI_PROV_SCHEME_HANDLER_NONE,
                          WIFI_PROV_SECURITY_1,
                          pop,
                          service_name);
}

// Main loop
void loop() {
  button1.check();
  button2.check();
  button3.check();
  button4.check();
  fan_button.check();

  // Factory reset on long press
  if (digitalRead(GPIO_RESET) == LOW) {
    unsigned long start = millis();
    while (digitalRead(GPIO_RESET) == LOW && (millis() - start) < 6000) delay(50);
    if ((millis() - start) >= 5000) {
      Serial.println("Factory reset triggered");
      RMakerFactoryReset(2);
    }
  }

  delay(50);
}
