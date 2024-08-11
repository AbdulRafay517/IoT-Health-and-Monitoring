#include <Wire.h>
#include <TinyGPSPlus.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Pin definitions
#define DHTPIN 2        // DHT22 data pin connected to ESP32 GPIO 2
#define DHTTYPE DHT22   // DHT 22 (AM2302)
#define NANO_RX 16      // Arduino Nano TX0 connected to ESP32 GPIO 16 (RX2)
#define NANO_TX 17      // Arduino Nano RX0 connected to ESP32 GPIO 17 (TX2)

// Firebase project credentials
#define FIREBASE_API_KEY "AIzaSyAtUxgcFIqxt9aXZSMd8xtp_GpFDx1IRJ4"
#define FIREBASE_DATABASE_URL "iot-health-monitoring-8654a-default-rtdb.asia-southeast1.firebasedatabase.app"

// WiFi credentials
#define WIFI_SSID "iotmonitor"
#define WIFI_PASSWORD "iotmonitor"

// Initialize sensors
TinyGPSPlus gps;
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;

HardwareSerial SerialNano(2);

float heartRate = 0;
float spo2 = 0;

// Firebase Data and Config objects
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// Variables to store sensor data
float temperature, humidity;
sensors_event_t accel, gyro, temp;

bool setupComplete=false;

void setup() {
  Serial.begin(115200);
  SerialNano.begin(9600, SERIAL_8N1, NANO_RX, NANO_TX);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
  Serial.println("DHT22 initialized");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 initialized");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Waiting for GPS signal...");

  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE_URL;

  Firebase.signUp(&config, &auth, "", "");

  if (Firebase.ready()) {
    Serial.println("Firebase connection established");
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    setupComplete = true;
    Serial.println("Setup complete. Starting main loop.");
  } else {
    Serial.println("Failed to connect to Firebase");
    Serial.println("Reason: " + fbdo.errorReason());
  }
 
}

void loop() {
  if (!setupComplete) {
    Serial.println("Setup incomplete. Main loop will not run.");
    delay(5000);  // Wait 5 seconds before checking again
    return;
  }

  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.printf("GPS: Lat: %.6f, Lon: %.6f, Alt: %.2f, Speed: %.2f\n",
                  gps.location.lat(), gps.location.lng(),
                  gps.altitude.meters(), gps.speed.kmph());

    Firebase.setFloat(fbdo, "/gps/latitude", gps.location.lat());
    Firebase.setFloat(fbdo, "/gps/longitude", gps.location.lng());
    Firebase.setFloat(fbdo, "/gps/altitude", gps.altitude.meters());
    Firebase.setFloat(fbdo, "/gps/speed", gps.speed.kmph());
  }

  if (SerialNano.available()) {
    String data = SerialNano.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    if (commaIndex != -1) {
      heartRate = data.substring(0, commaIndex).toFloat();
      spo2 = data.substring(commaIndex + 1).toFloat();
      
      Serial.printf("Heart Rate: %.1f bpm, SpO2: %.1f%%\n", heartRate, spo2);
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Pin definitions
#define DHTPIN 2        // DHT22 data pin connected to ESP32 GPIO 2
#define DHTTYPE DHT22   // DHT 22 (AM2302)
#define NANO_RX 16      // Arduino Nano TX0 connected to ESP32 GPIO 16 (RX2)
#define NANO_TX 17      // Arduino Nano RX0 connected to ESP32 GPIO 17 (TX2)

// Firebase project credentials
#define FIREBASE_API_KEY "AIzaSyAtUxgcFIqxt9aXZSMd8xtp_GpFDx1IRJ4"
#define FIREBASE_DATABASE_URL "iot-health-monitoring-8654a-default-rtdb.asia-southeast1.firebasedatabase.app"

// WiFi credentials
#define WIFI_SSID "iotmonitor"
#define WIFI_PASSWORD "iotmonitor"

// Initialize sensors
TinyGPSPlus gps;
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;

HardwareSerial SerialNano(2);

float heartRate = 0;
float spo2 = 0;

// Firebase Data and Config objects
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// Variables to store sensor data
float temperature, humidity;
sensors_event_t accel, gyro, temp;

bool setupComplete=false;

void setup() {
  Serial.begin(115200);
  SerialNano.begin(9600, SERIAL_8N1, NANO_RX, NANO_TX);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  dht.begin();
  Serial.println("DHT22 initialized");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 initialized");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Waiting for GPS signal...");

  config.api_key = FIREBASE_API_KEY;
  config.database_url = FIREBASE_DATABASE_URL;

  Firebase.signUp(&config, &auth, "", "");

  if (Firebase.ready()) {
    Serial.println("Firebase connection established");
    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    setupComplete = true;
    Serial.println("Setup complete. Starting main loop.");
  } else {
    Serial.println("Failed to connect to Firebase");
    Serial.println("Reason: " + fbdo.errorReason());
  }
 
}

void loop() {
  if (!setupComplete) {
    Serial.println("Setup incomplete. Main loop will not run.");
    delay(5000);  // Wait 5 seconds before checking again
    return;
  }

  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.printf("GPS: Lat: %.6f, Lon: %.6f, Alt: %.2f, Speed: %.2f\n",
                  gps.location.lat(), gps.location.lng(),
                  gps.altitude.meters(), gps.speed.kmph());

    Firebase.setFloat(fbdo, "/gps/latitude", gps.location.lat());
    Firebase.setFloat(fbdo, "/gps/longitude", gps.location.lng());
    Firebase.setFloat(fbdo, "/gps/altitude", gps.altitude.meters());
    Firebase.setFloat(fbdo, "/gps/speed", gps.speed.kmph());
  }

  if (SerialNano.available()) {
    String data = SerialNano.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    if (commaIndex != -1) {
      heartRate = data.substring(0, commaIndex).toFloat();
      spo2 = data.substring(commaIndex + 1).toFloat();
      
      Serial.printf("Heart Rate: %.1f bpm, SpO2: %.1f%%\n", heartRate, spo2);

      Firebase.setFloat(fbdo, "/health/heart_rate", heartRate);
      Firebase.setFloat(fbdo, "/health/spo2", spo2);
      }
    }

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.printf("Temp: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);

    Firebase.setFloat(fbdo, "/environment/temperature", temperature);
    Firebase.setFloat(fbdo, "/environment/humidity", humidity);
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  mpu.getEvent(&accel, &gyro, &temp);

  Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f m/s^2\n",
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  Serial.printf("Gyro X: %.2f, Y: %.2f, Z: %.2f rad/s\n",
                gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

  Firebase.setFloat(fbdo, "/motion/accel_x", accel.acceleration.x);
  Firebase.setFloat(fbdo, "/motion/accel_y", accel.acceleration.y);
  Firebase.setFloat(fbdo, "/motion/accel_z", accel.acceleration.z);
  Firebase.setFloat(fbdo, "/motion/gyro_x", gyro.gyro.x);
  Firebase.setFloat(fbdo, "/motion/gyro_y", gyro.gyro.y);
  Firebase.setFloat(fbdo, "/motion/gyro_z", gyro.gyro.z);

  Serial.println();

  delay(100);
}
      Firebase.setFloat(fbdo, "/health/heart_rate", heartRate);
      Firebase.setFloat(fbdo, "/health/spo2", spo2);
      }
    }

  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.printf("Temp: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);

    Firebase.setFloat(fbdo, "/environment/temperature", temperature);
    Firebase.setFloat(fbdo, "/environment/humidity", humidity);
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  mpu.getEvent(&accel, &gyro, &temp);

  Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f m/s^2\n",
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  Serial.printf("Gyro X: %.2f, Y: %.2f, Z: %.2f rad/s\n",
                gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

  Firebase.setFloat(fbdo, "/motion/accel_x", accel.acceleration.x);
  Firebase.setFloat(fbdo, "/motion/accel_y", accel.acceleration.y);
  Firebase.setFloat(fbdo, "/motion/accel_z", accel.acceleration.z);
  Firebase.setFloat(fbdo, "/motion/gyro_x", gyro.gyro.x);
  Firebase.setFloat(fbdo, "/motion/gyro_y", gyro.gyro.y);
  Firebase.setFloat(fbdo, "/motion/gyro_z", gyro.gyro.z);

  Serial.println();

  delay(100);
}