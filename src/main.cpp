#include <Wire.h>
#include <vector>
#include <iostream>
#include "Adafruit_AHTX0.h"  // Library for AHT20/DHT20
#include <Arduino.h>
#include <HttpClient.h>
#include <WiFi.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

// WiFi credentials
const char* WIFI_SSID = "si-utility"; // Network SSID
const char* WIFI_PASSWORD = "whatyearisit"; // Network password

// Server details
const char kHostname[] = "34.254.178.229";
const char kPath[] = "/"; // Path to send data
const int kPort = 5000;

// Sensor object
Adafruit_AHTX0 aht;

// Define I2C pins
#define SDA_PIN 33
#define SCL_PIN 32  // Update this with the GPIO pin you'll connect for SCL

// data store
bool isTempSensorInitialized = false;
std::vector<float> temperatures = {17.3, 18, 17.9};
std::vector<float> humidities = {61.1, 61.5};

void setupWIfiConnection();

bool setupTempHumiditySensor();

void measureAndSendTempHumidityData();

float getMean(const std::vector<float>& elements);

// WiFi connection setup
void setup() {
    Serial.begin(9600);
    delay(1000);

    setupWIfiConnection();
    setupTempHumiditySensor();
}

// Loop to read sensor data and send to server
void loop() {
    measureAndSendTempHumidityData();
    delay(10000);  // Wait 10 seconds before sending again
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

void measureAndSendTempHumidityData(){
    // Read temperature and humidity
    if (isTempSensorInitialized || setupTempHumiditySensor()) {
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);  // Read data into the event objects
        if (!isnan(temp.temperature) && !isnan(humidity.relative_humidity)){
            temperatures.erase(temperatures.begin());
            humidities.erase(humidities.begin());

            temperatures.push_back(temp.temperature);
            humidities.push_back(humidity.relative_humidity);
            Serial.printf("Measured temp=%f \t humidity=%f\n", temp.temperature, humidity.relative_humidity);
        }
    }
    float temperature = getMean(temperatures);
    float humidity = getMean(humidities);

    // Format the data as JSON
    String payload = "{";
    payload += "\"temperature\": " + String(temperature, 1) + ",";
    payload += "\"humidity\": " + String(humidity, 1);
    payload += "}";

    // Send data to the server
    WiFiClient c;
    HttpClient http(c, kHostname, kPort);  // Corrected constructor

    Serial.println("Sending data to server...");
    int err = http.post(kPath, "application/json", payload.c_str());  // Corrected method
    if (err == 0) {
        int statusCode = http.responseStatusCode();
        Serial.print("Response status code: ");
        Serial.println(statusCode);

        if (statusCode == 200) {
            Serial.println("Data sent successfully.");
        } else {
            Serial.println("Failed to send data.");
        }
    } else {
        Serial.print("HTTP POST failed: ");
        Serial.println(err);
    }

    http.stop();
}

float getMean(const std::vector<float> &elements) {
    float sum = 0;
    for (const auto element: elements) {
        sum += element;
    }
    return sum / elements.size();
}
