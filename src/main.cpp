#include <Wire.h>
#include <vector>
#include <iostream>
#include <ctime>
#include "Adafruit_AHTX0.h"  // Library for AHT20/DHT20
#include <Arduino.h>
#include <HttpClient.h>
#include <WiFi.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "Servo.h"
#include "Adafruit_ST77xx.h"
#include "Adafruit_ST7789.h"

// PINs
#define SERVO_PIN 27
#define SOIL_MOISTURE_PIN 34
// Define I2C pins for temp-humidity sensor
#define SDA_PIN 33
#define SCL_PIN 32  // Update this with the GPIO pin you'll connect for SCL


// Constants
#define MIN_SERVO_ROTATION 0
#define MAX_SERVO_ROTATION 180
#define WATER_IF_SOIL_MOISTURE 2000

// LCD constants
#define LCD_CS 15
#define LCD_DC 2
#define LCD_RST 4

// WiFi credentials
const char *WIFI_SSID = "YOUR_SSID_HERE"; // Network SSID
const char *WIFI_PASSWORD = "YOUR_PASSWORD_HERE"; // Network password

// Server details
const IPAddress kHostname = IPAddress(52, 9, 30, 235);
const char kPath[] = "/plant-sensor-data/"; // Path to send data
const int kPort = 5000;

// Sensor object
Adafruit_AHTX0 aht;
Servo servo;
Adafruit_ST7789 lcd = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

// data store
bool isTempSensorInitialized = false;
std::vector<float> temperatures;
std::vector<float> humidities;
bool is_watering = false;
float temperature;
float humidity;
int moisture;

void setupServo();

void setupWIfiConnection();

void setupSoilMoistureSensor();

bool setupTempHumiditySensor();

void setupLcd();

void measureTempHumidityData();

float getMean(const std::vector<float> &elements);

void water_plant(bool should_water);

void renderPlantWateringDrawing();

void renderPlantNotBeingWateredDrawing();

bool measureMoistureAndDecideShouldWater();

void sendDataToCloud();

// WiFi connection setup
void setup() {
    Serial.begin(9600);
    delay(1000);

    setupLcd();
    setupServo();
    setupSoilMoistureSensor();
    setupWIfiConnection();
    setupTempHumiditySensor();
}

void setupSoilMoistureSensor() {
    pinMode(SOIL_MOISTURE_PIN, INPUT);
}

// Loop to read sensor data and send to server
void loop() {
    measureTempHumidityData();
    measureMoistureAndDecideShouldWater();
    sendDataToCloud();
    water_plant(is_watering);
    delay(3000);  // Wait 10 seconds before sending aain
}

void setupLcd() {
    lcd.init(135, 240);
    lcd.fillScreen(ST77XX_BLACK);
}

void setupServo() {
    servo.attach(SERVO_PIN);
    Serial.println("Initialized servo motor");
}

void setupWIfiConnection() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

bool setupTempHumiditySensor() {
    // Initialize I2C with custom pins
    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println("Initializing temp humidity sensor");
    if (aht.begin()) {
        isTempSensorInitialized = true;
        Serial.println("AHT20 sensor initialized.");
    } else {
        Serial.println("Unable to initialize Temp Humidity sensor. Check wiring");
    }
    return isTempSensorInitialized;
}

void measureTempHumidityData() {
    // Read temperature and humidity
    if (isTempSensorInitialized || setupTempHumiditySensor()) {
        sensors_event_t humidity_raw, temp;
        aht.getEvent(&humidity_raw, &temp);  // Read data into the event objects
        if (!isnan(temp.temperature) && !isnan(humidity_raw.relative_humidity)) {
            temperatures.erase(temperatures.begin());
            humidities.erase(humidities.begin());

            temperatures.push_back(temp.temperature);
            humidities.push_back(humidity_raw.relative_humidity);
            Serial.printf("Measured temp=%f \t humidity_raw=%f\n", temp.temperature, humidity_raw.relative_humidity);
        }
    }
    temperature = getMean(temperatures);
    humidity = getMean(humidities);
}

void water_plant(bool should_water) {
    if (should_water) {
        servo.write(MAX_SERVO_ROTATION);
        Serial.println("Watering plant");
        renderPlantWateringDrawing();
    } else {
        servo.write(MIN_SERVO_ROTATION);
        Serial.println("Not watering plant");
        renderPlantNotBeingWateredDrawing();
    }
}

void renderPlantWateringDrawing() {
    // Background
    lcd.fillScreen(ST77XX_CYAN);

    // Draw soil
    lcd.fillRect(40, 180, 60, 40, ST77XX_BROWN);

    // Draw plant stem
    lcd.fillRect(70, 140, 5, 40, ST77XX_GREEN);

    // Draw leaves
    lcd.fillTriangle(65, 140, 75, 120, 85, 140, ST77XX_GREEN);
    lcd.fillTriangle(60, 150, 75, 130, 90, 150, ST77XX_GREEN);

    // Draw water droplets
    lcd.fillCircle(50, 60, 5, ST77XX_BLUE);
    lcd.fillCircle(60, 80, 4, ST77XX_BLUE);
    lcd.fillCircle(70, 100, 3, ST77XX_BLUE);
    lcd.fillCircle(80, 120, 2, ST77XX_BLUE);

    // Draw watering can
    lcd.fillRect(40, 40, 30, 15, ST77XX_RED);
    lcd.fillRect(60, 30, 8, 8, ST77XX_RED);
    lcd.fillRect(70, 35, 15, 5, ST77XX_RED);
}

bool measureMoistureAndDecideShouldWater() {
    moisture = (int) analogRead(SOIL_MOISTURE_PIN);
    Serial.printf("Soil moisture value = %d\n", moisture);
    return moisture > WATER_IF_SOIL_MOISTURE;
}

void sendDataToCloud() {
    // Format the data as JSON
    String payload = "{";
    payload += "\"temperature\": " + String(temperature, 1) + ",";
    payload += "\"humidity\": " + String(humidity, 1) + ",";
    payload += "\"moisture\": " + String(moisture);
    payload += "}";

    // Send data to the server
    WiFiClient c;
    HttpClient http(c, kHostname, kPort);  // Corrected constructor

    Serial.println("Sending data to server...");
    Serial.println(payload);
    int err = http.post(kPath, "application/json", payload.c_str());
    if (err == 0) {
        int statusCode = http.responseStatusCode();
        if (statusCode == 200) {
            const char* response_body = http.responseBody().c_str();
            Serial.println("Data sent successfully.");
            Serial.println("Response body:");
            Serial.println(response_body);
            const char* should_water_true = "true";
            if (strstr(response_body, should_water_true) != nullptr) {
                is_watering = true;
            } else{
                is_watering = false;
            }
        } else {
            Serial.println("Failed to send data.");
        }
        Serial.print("Response status code: ");
        Serial.println(statusCode);
    } else {
        Serial.print("HTTP POST failed: ");
        Serial.println(err);
    }
    http.stop();
}

void renderPlantNotBeingWateredDrawing() {
    // Background
    lcd.fillScreen(ST77XX_CYAN);

    // Draw soil
    lcd.fillRect(40, 180, 60, 40, ST77XX_BROWN);

    // Draw plant stem
    lcd.fillRect(70, 140, 5, 40, ST77XX_GREEN);

    // Draw leaves
    lcd.fillTriangle(65, 140, 75, 120, 85, 140, ST77XX_GREEN);
    lcd.fillTriangle(60, 150, 75, 130, 90, 150, ST77XX_GREEN);

    // No water droplets

    // Draw empty watering can far from the plant
    lcd.fillRect(10, 40, 30, 15, ST77XX_RED);
    lcd.fillRect(30, 30, 8, 8, ST77XX_RED);
    lcd.fillRect(40, 35, 15, 5, ST77XX_RED);
}

float getMean(const std::vector<float> &elements) {
    float sum = 0;
    for (const auto element: elements) {
        sum += element;
    }
    return sum / elements.size();
}
