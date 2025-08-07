#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define LED_PIN D7
#define BUTTON_PIN D6
#define SDA_PIN 4  // D2
#define SCL_PIN 5  // D1

// Wi-Fi credentials
const char *ssid = "";
const char *password = "";

// MQTT configuration
const char *mqtt_server = "192.168.1.28";
String mqtt_client_id = "sensor_living_room";

WiFiClient client;
PubSubClient subClient(client);

// Sensors
BH1750 lightMeter;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Timing
unsigned long previousMillis = 0;
const unsigned long interval = 60000; // 60 seconds

// Button state
bool lastButtonState = HIGH;

void blinkLED(int times = 1, int duration = 200) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    delay(duration);
  }
}

void connectWiFi() {
  Serial.println("[INFO] Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[INFO] Wi-Fi connected");
  Serial.print("[INFO] IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!subClient.connected()) {
    Serial.println("[INFO] Attempting MQTT connection...");
    if (subClient.connect(mqtt_client_id.c_str())) {
      Serial.print("[INFO] Connected to MQTT broker as ");
      Serial.println(mqtt_client_id);
    } else {
      Serial.print("[ERROR] MQTT connection failed. RC = ");
      Serial.println(subClient.state());
      delay(2000);
    }
  }
}

bool publishSensorData(const char *topic, StaticJsonDocument<200> &doc) {
  char buffer[200];
  serializeJson(doc, buffer);
  Serial.print("[MQTT] Publishing to topic '");
  Serial.print(topic);
  Serial.print("': ");
  Serial.println(buffer);

  if (subClient.publish(topic, buffer)) {
    blinkLED();
    Serial.println("[MQTT] Message published successfully.");
    return true;
  } else {
    Serial.println("[ERROR] Failed to publish message.");
    return false;
  }
}

void readAndSendSensors() {
  // Read light level
  float lux = lightMeter.readLightLevel();
  Serial.print("[SENSOR] Light level: ");
  Serial.print(lux);
  Serial.println(" lx");

  // Read pressure and temperature
  sensors_event_t event;
  bmp.getEvent(&event);
  float temperature = 0;
  float pressure = 0;

  if (event.pressure) {
    bmp.getTemperature(&temperature);
    pressure = event.pressure;
    Serial.print("[SENSOR] Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("[SENSOR] Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
  } else {
    Serial.println("[ERROR] Failed to read BMP180 sensor.");
  }

  // Message 1: Light
  StaticJsonDocument<200> msg1;
  msg1["sensor_id"] = mqtt_client_id;
  msg1["ip_address"] = WiFi.localIP().toString();
  msg1["value"] = lux;
  publishSensorData("appartment/livingroom/lux", msg1);

  // Message 2: Temperature
  StaticJsonDocument<200> msg2;
  msg2["sensor_id"] = mqtt_client_id;
  msg2["ip_address"] = WiFi.localIP().toString();
  msg2["value"] = temperature;
  publishSensorData("appartment/livingroom/temperature", msg2);

  // Message 3: Temperature
  StaticJsonDocument<200> msg3;
  msg3["sensor_id"] = mqtt_client_id;
  msg3["ip_address"] = WiFi.localIP().toString();
  msg3["value"] = pressure;
  publishSensorData("appartment/livingroom/pressure", msg3);

  Serial.println("[INFO] Sensor data sent.");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("");
  Serial.println("====================================");
  Serial.println("Domo sensor by ARZ - 06/08/2025");
  Serial.println("====================================");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button connected between D6 and GND

  connectWiFi();
  blinkLED(3, 300);

  subClient.setServer(mqtt_server, 1883);
  subClient.setCallback(mqtt_callback);

  Serial.println("[INFO] Initializing I2C bus...");
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("[INFO] Initializing BH1750 light sensor...");
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("[ERROR] Failed to initialize BH1750.");
    while (1);
  }
  Serial.println("[INFO] BH1750 initialized successfully.");

  Serial.println("[INFO] Initializing BMP180 pressure sensor...");
  if (!bmp.begin()) {
    Serial.println("[ERROR] Failed to initialize BMP180.");
    while (1);
  }
  Serial.println("[INFO] BMP180 initialized successfully.");
}

void loop() {
  if (!subClient.connected()) {
    mqttReconnect();
  }
  subClient.loop();

  // Timer-based reading
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readAndSendSensors();
  }

  // Button-based manual sending
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    Serial.println("[BUTTON] Button pressed - sending data now...");
    readAndSendSensors();
  }
  lastButtonState = currentButtonState;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("[MQTT] Message received on topic: ");
  Serial.println(topic);
}
