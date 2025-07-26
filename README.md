# Smart-Cabin-Automation
Overview
This project implements a smart home automation system based on the ESP32 microcontroller. It controls four independent lighting circuits and a ceiling fan (ON/OFF at max speed) through both physical wall switches and the ESP RainMaker cloud platform. It provides seamless integration of local manual control and remote app control with reliable state synchronization.

Features
Control four lighting circuits independently via wall-mounted switches or remote app.

Control a ceiling fan with ON/OFF functionality via a single relay.

Integration with ESP RainMaker IoT platform for secure cloud control and OTA updates.

Local user input handling via debounced physical switches using AceButton library.

BLE WiFi provisioning with BLE QR code for easy setup.

Factory reset via a long press button for easy re-provisioning.

Reliable and robust relay control with hardware isolation and safe switching logic.

Serial debug output for easy development and troubleshooting.
