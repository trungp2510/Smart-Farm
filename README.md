# 🌱 Smart Farm Monitoring System

A real-time environmental monitoring and remote control system for smart farming using **ESP32**, **ThingsBoard**, **DHT20**, and other sensors.

## 📦 Features

- 🌡️ Reads **temperature** and **humidity** using DHT20.
- 💧 Monitors **soil moisture** and **light intensity**.
- 📺 Displays data on **I2C LCD (16x2)**.
- ☁️ Sends real-time telemetry data to **ThingsBoard** over MQTT.
- 🧠 Supports **remote control** of LED, fan, and pump via RPC.
- 🔄 Receives shared attributes (auto control from dashboard).
- 🔧 OTA-ready with `ArduinoOTA` integration (optional).

## 🧰 Hardware Used

| Device              | Description                     |
|---------------------|----------------------------------|
| ESP32               | Main controller (WiFi + RTOS)   |
| DHT20               | Temperature and humidity sensor |
| Soil Moisture Sensor| Analog soil moisture detection  |
| Light Sensor        | Analog light intensity input    |
| LCD I2C (16x2)      | Displays sensor data            |
| Relay/LED/Fan/Pump  | Controlled actuators            |

## 📡 Cloud Platform

- [CoreIoT Platform](https://coreiot.io/)
  - Used for telemetry visualization, remote RPC control, and attribute updates.

## 🔗 Wiring Overview

| ESP32 Pin     | Device               |
|---------------|----------------------|
| GPIO 1        | Soil Moisture Sensor |
| GPIO 2        | Light Sensor         |
| GPIO 5        | LED                  |
| GPIO 6        | Fan                  |
| GPIO 7        | Pump                 |
| GPIO 11 (SDA) | I2C LCD & DHT20      |
| GPIO 12 (SCL) | I2C LCD & DHT20      |

## ⚙️ Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/smart-farm-esp32.git
   cd smart-farm-esp32
