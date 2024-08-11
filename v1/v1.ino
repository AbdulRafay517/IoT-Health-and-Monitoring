#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MAX30105.h>
#include <Adafruit_GPS.h>
#include <DHT.h>
#include <MPU6050.h>

#define DHTPIN 4          // Pin for DHT22
#define DHTTYPE DHT22     // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);

// Initialize MAX30105 sensor
MAX30105 max30105;
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Initialize MPU6050 sensor
MPU6050 mpu6050;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Initialize GPS sensor
HardwareSerial mySerial(1);
Adafruit_GPS GPS(&mySerial);

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C begin");

  // Initialize DHT22 sensor
  dht.begin();
  Serial.println("DHT22 begin");

  // Initialize MPU6050 sensor
  mpu6050.initialize();
  if (!mpu6050.testConnection()) {
    Serial.println("MPU6050 not found. Please check wiring/power.");
    while (1);
  }
  Serial.println("MPU6050 begin");

  // Initialize MAX30105 sensor
  if (!max30105.begin(Wire, I2C_SDA, I2C_SCL)) {
    Serial.println("MAX30105 not found. Please check wiring/power.");
    while (1);
  }
  Serial.println("MAX30105 begin");
  max30105.setup(); // Configure sensor with default settings

  // Initialize GPS sensor
  mySerial.begin(9600, SERIAL_8N1, 16, 17);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  Serial.println("Setup complete.");
}

void loop() {
  // Read DHT22 sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Read MAX30105 sensor
  uint32_t irValue = max30105.getIR();

  // Read MPU6050 sensor
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read GPS sensor
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  // Output readings to serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" *C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, IR Value: ");
  Serial.print(irValue);
  Serial.print(", Ax: ");
  Serial.print(ax);
  Serial.print(", Ay: ");
  Serial.print(ay);
  Serial.print(", Az: ");
  Serial.print(az);
  Serial.print(", Gx: ");
  Serial.print(gx);
  Serial.print(", Gy: ");
  Serial.print(gy);
  Serial.print(", Gz: ");
  Serial.print(gz);
  Serial.print(", GPS: ");
  if (GPS.fix) {
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
  } else {
    Serial.print("No fix");
  }
  Serial.println();

  delay(1000);  // Update every second
}