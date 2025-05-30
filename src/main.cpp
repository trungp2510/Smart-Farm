#define LED_PIN GPIO_NUM_5
#define FAN_PIN GPIO_NUM_6
#define PUMP_PIN GPIO_NUM_7
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define SOIL_DETECTOR GPIO_NUM_1
#define LIGHT_DETECTOR GPIO_NUM_2


#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include "LiquidCrystal_I2C.h"
#include <time.h>
// constexpr char WIFI_SSID[] = "BiBo";
// constexpr char WIFI_PASSWORD[] = "trunghieu3868";

constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";

constexpr char TOKEN[] = "l7a0izvyspa9ewse5vde";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 16384U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";
constexpr char FAN_STATE_ATTR[] = "pumpState";
constexpr char PUMP_STATE_ATTR[] = "fanState";

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;
volatile bool pumpState = false;
volatile bool fanState = false;


constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, 4U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR,
  FAN_STATE_ATTR,             
  PUMP_STATE_ATTR  
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
LiquidCrystal_I2C lcd(0x27,16,2);

RPC_Response setLedSwitchState(const RPC_Data &data) {
    Serial.println("Received Led state");
    bool newState = data["KeyLed"];
    Serial.print("Led state change: ");
    Serial.println(newState);
    digitalWrite(LED_PIN, newState);
    tb.sendAttributeData("ledstate",newState);
    return RPC_Response("setLedSwitchValue", newState);
}
RPC_Response setPumpSwitchState(const RPC_Data &data) {
    Serial.println("Received Pump state");
    bool newState = data["KeyPump"];
    Serial.print("Pump state change: ");
    Serial.println(newState);
    digitalWrite(PUMP_PIN, newState); 
    tb.sendAttributeData("pumpstate", newState);
    return RPC_Response("setPumpSwitchValue", newState);
}

RPC_Response setFanSwitchState(const RPC_Data &data) {
    Serial.println("Received Fan state");
    bool newState = data["KeyFan"];
    Serial.print("Fan state change: ");
    Serial.println(newState);
    digitalWrite(FAN_PIN, newState);
    tb.sendAttributeData("fanstate",newState);
    return RPC_Response("setFanSwitchValue", newState);
}

const std::array<RPC_Callback, 3U> callbacks = {
  RPC_Callback{ "LedState", setLedSwitchState},
  RPC_Callback{ "FanState", setFanSwitchState},
  RPC_Callback{ "PumpState", setPumpSwitchState}
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
        if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
            uint16_t interval = it->value().as<uint16_t>();
            if (interval >= BLINKING_INTERVAL_MS_MIN && interval <= BLINKING_INTERVAL_MS_MAX) {
                blinkingInterval = interval;
                Serial.print("Blinking interval is set to: ");
                Serial.println(interval);
            }
        } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
            ledState = it->value().as<bool>();
            digitalWrite(LED_PIN, ledState);
            Serial.print("LED state is set to: ");
            Serial.println(ledState);
        } else if (strcmp(it->key().c_str(), FAN_STATE_ATTR) == 0) {
            fanState = it->value().as<bool>();
            digitalWrite(FAN_PIN, fanState);
            Serial.print("Fan state is set to: ");
            Serial.println(fanState);
        } else if (strcmp(it->key().c_str(), PUMP_STATE_ATTR) == 0) {
            pumpState = it->value().as<bool>();
            digitalWrite(PUMP_PIN, pumpState);
            Serial.print("Pump state is set to: ");
            Serial.println(pumpState);
        }
    }
    attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to AP ...");
// Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}


void WiFiTask(void *pvParameters) {
  while(1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("WiFi reconnected");
    }
    vTaskDelay(10000);
  }
}

void serverTask(void *pvParameters) {
  while(1) {
    if (!tb.connected()) {
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        return;
      }
  
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
  
      Serial.println("Subscribing for RPC...");
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        return;
      }
  
      if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
        Serial.println("Failed to subscribe for shared attribute updates");
        return;
      }
  
      Serial.println("Subscribe done");
  
      if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
        Serial.println("Failed to request for shared attributes");
        return;
      }
    }
    vTaskDelay(1000);
  }
}
void LoopTask(void * pvParameters) {
  while(1) {
    tb.loop();
    vTaskDelay(1000);
  }
}
void LCDTask(void *pvParameters) {
  while(1) {
    float soil_mois_value = analogRead(SOIL_DETECTOR);
    int light_value = analogRead(LIGHT_DETECTOR);
    int converted_soil_value = (1-(soil_mois_value/4095))*100;
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RT:");
    lcd.setCursor(3,0);
    lcd.print(temperature);
    lcd.setCursor(7,0);
    lcd.print("*C");
    lcd.setCursor(10,0);
    lcd.print("RH:");
    lcd.setCursor(13,0);
    lcd.print(humidity);
    lcd.setCursor(15,0);
    lcd.print("%");
    lcd.setCursor(0,1);
    lcd.print("LUX:");
    lcd.setCursor(4,1);
    lcd.print(light_value);
    lcd.setCursor(10,1);
    lcd.print("SM:");
    lcd.setCursor(13,1);
    lcd.print(converted_soil_value);
    lcd.setCursor(15,1);
    lcd.print("%");
    vTaskDelay(5000);
  }
}
void sendTask(void * pvParameters) {
  while (1) {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();
    float soil_mois_value = analogRead(SOIL_DETECTOR);
    float light_value = analogRead(LIGHT_DETECTOR);
    float converted_soil_value = (1 - (soil_mois_value / 4095.0)) * 100;

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT20 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    }

    Serial.print("Soil-Moisture: ");
    Serial.print(converted_soil_value);
    Serial.println(" %");
    Serial.print("Light: ");
    Serial.print(light_value);
    Serial.println(" LUX");

    // Gửi tất cả dữ liệu cùng lúc dưới dạng JSON (kèm status)
    String jsonPayload = "{";
    jsonPayload += "\"env_temp\":" + String(temperature, 2) + ",";
    jsonPayload += "\"env_humi\":" + String(humidity, 2) + ",";
    jsonPayload += "\"env_light\":" + String(light_value, 2) + ",";
    jsonPayload += "\"env_soil\":" + String(converted_soil_value, 2) + ",";
    jsonPayload += "\"status\":\"normal\"";
    jsonPayload += "}";

    tb.sendTelemetryJson(jsonPayload.c_str());

    vTaskDelay(5000);
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("Smart Farm");
  delay(3000);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  InitWiFi();
  dht20.begin();

  xTaskCreate(WiFiTask, "WiFiTask", 4096, NULL, 1, NULL);
  xTaskCreate(serverTask, "serverTask", 8172, NULL, 1, NULL);
  xTaskCreate(LoopTask, "LoopTask", 4096, NULL, 1, NULL);
  xTaskCreate(sendTask, "sendTask", 4096, NULL, 2, NULL);
  xTaskCreate(LCDTask, "LCDTask", 4096, NULL, 3, NULL);

}

void loop() {
  
}