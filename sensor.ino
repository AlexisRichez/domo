#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
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
Adafruit_BME680 bme680;

// Timing
unsigned long previousMillis = 0;
const unsigned long interval = 60000; // 60 seconds

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
  Serial.printf("[MQTT] Publishing to '%s': %s\n", topic, buffer);

  if (subClient.publish(topic, buffer)) {
    blinkLED();
    return true;
  }
  Serial.println("[ERROR] MQTT publish failed.");
  return false;
}

// --- Calculate simplified IAQ ---
float calculateIAQ(float gas_resistance, float humidity) {
  float gas_score = (log(gas_resistance) - log(1000)) / (log(1000000) - log(1000));
  gas_score = constrain(gas_score, 0.0, 1.0);

  float hum_score;
  if (humidity >= 40 && humidity <= 60) hum_score = 1.0;
  else if (humidity < 40) hum_score = humidity / 40.0;
  else hum_score = (100 - humidity) / 40.0;
  hum_score = constrain(hum_score, 0.0, 1.0);

  float iaq_score = (gas_score * 0.75 + hum_score * 0.25) * 500.0;
  return iaq_score;
}

// --- Interpret IAQ in English ---
const char* interpretIAQ(float iaq) {
  if (iaq <= 50) return "Excellent";
  else if (iaq <= 100) return "Good";
  else if (iaq <= 150) return "Moderate";
  else if (iaq <= 200) return "Poor";
  else if (iaq <= 300) return "Very Poor";
  else return "Severe";
}

void readAndSendSensors() {
  // --- BH1750 ---
  float lux = lightMeter.readLightLevel();
  Serial.printf("[SENSOR] Light: %.2f lx\n", lux);
  StaticJsonDocument<200> msg1;
  msg1["sensor_id"] = mqtt_client_id;
  msg1["ip_address"] = WiFi.localIP().toString();
  msg1["value"] = lux;
  publishSensorData("appartment/livingroom/lux", msg1);

  // --- BMP180 ---
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    float temperature, pressure;
    bmp.getTemperature(&temperature);
    pressure = event.pressure;
    Serial.printf("[SENSOR] BMP180 Temp: %.2f °C | Pressure: %.2f hPa\n", temperature, pressure);

    StaticJsonDocument<200> msg2;
    msg2["sensor_id"] = mqtt_client_id;
    msg2["ip_address"] = WiFi.localIP().toString();
    msg2["value"] = temperature;
    publishSensorData("appartment/livingroom/temperature", msg2);

    StaticJsonDocument<200> msg3;
    msg3["sensor_id"] = mqtt_client_id;
    msg3["ip_address"] = WiFi.localIP().toString();
    msg3["value"] = pressure;
    publishSensorData("appartment/livingroom/pressure", msg3);
  }

  // --- BME680 ---
  if (bme680.performReading()) {
    float temp = bme680.temperature;
    float hum = bme680.humidity;
    float press = bme680.pressure / 100.0;
    float gas = bme680.gas_resistance;
    float iaq = calculateIAQ(gas, hum);
    const char *iaq_text = interpretIAQ(iaq);

    Serial.printf("[SENSOR] BME680 Temp: %.2f °C | Hum: %.2f %% | Press: %.2f hPa | Gas: %.0f Ω | IAQ: %.1f (%s)\n",
                  temp, hum, press, gas, iaq, iaq_text);

    StaticJsonDocument<200> msgT;
    msgT["sensor_id"] = mqtt_client_id;
    msgT["ip_address"] = WiFi.localIP().toString();
    msgT["value"] = temp;
    publishSensorData("appartment/livingroom/bme680/temperature", msgT);

    StaticJsonDocument<200> msgH;
    msgH["sensor_id"] = mqtt_client_id;
    msgH["ip_address"] = WiFi.localIP().toString();
    msgH["value"] = hum;
    publishSensorData("appartment/livingroom/humidity", msgH);

    StaticJsonDocument<200> msgP;
    msgP["sensor_id"] = mqtt_client_id;
    msgP["ip_address"] = WiFi.localIP().toString();
    msgP["value"] = press;
    publishSensorData("appartment/livingroom/bme680/pressure", msgP);

    StaticJsonDocument<200> msgG;
    msgG["sensor_id"] = mqtt_client_id;
    msgG["ip_address"] = WiFi.localIP().toString();
    msgG["value"] = gas;
    publishSensorData("appartment/livingroom/gas", msgG);

    StaticJsonDocument<200> msgIAQ;
    msgIAQ["sensor_id"] = mqtt_client_id;
    msgIAQ["ip_address"] = WiFi.localIP().toString();
    msgIAQ["value"] = iaq;
    publishSensorData("appartment/livingroom/iaq", msgIAQ);

    StaticJsonDocument<200> msgIAQQuality;
    msgIAQ["sensor_id"] = mqtt_client_id;
    msgIAQ["ip_address"] = WiFi.localIP().toString();
    msgIAQ["value"] = iaq_text;
    publishSensorData("appartment/livingroom/iaq/quality", msgIAQ);

  } else {
    Serial.println("[ERROR] Failed to perform BME680 reading.");
  }

  Serial.println("[INFO] Sensor data sent.");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n====================================");
  Serial.println("Domo sensor by ARZ");
  Serial.println("====================================");

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  connectWiFi();
  blinkLED(3, 300);

  subClient.setServer(mqtt_server, 1883);
  subClient.setCallback(mqtt_callback);

  Serial.println("[INFO] Initializing I2C...");
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("[INFO] Initializing BH1750...");
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("[ERROR] BH1750 init failed."); while (1);
  }

  Serial.println("[INFO] Initializing BMP180...");
  if (!bmp.begin()) {
    Serial.println("[ERROR] BMP180 init failed."); while (1);
  }

  Serial.println("[INFO] Initializing BME680...");
  if (!bme680.begin(0x76)) {
    Serial.println("[ERROR] BME680 init failed!");
  } else {
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);
    Serial.println("[INFO] BME680 ready.");
  }
}

void loop() {
  if (!subClient.connected()) mqttReconnect();
  subClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readAndSendSensors();
  }

  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    Serial.println("[BUTTON] Manual trigger - sending data now...");
    readAndSendSensors();
  }
  lastButtonState = currentButtonState;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("[MQTT] Message received on topic: ");
  Serial.println(topic);
}
