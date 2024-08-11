# IoT-Based Health and Environment Monitoring System

## Overview

This project is an IoT-based health and environment monitoring system using the ESP32 microcontroller. It collects data from various sensors, including a DHT22 temperature and humidity sensor, an MPU6050 accelerometer and gyroscope, a GPS module, and a MAX30100 pulse oximeter, and then uploads the data to a Firebase Realtime Database for real-time monitoring.

## Features

- **GPS Tracking:** Continuously monitors and uploads GPS data, including latitude, longitude, altitude, and speed.
- **Health Monitoring:** Measures heart rate and SpO2 levels using a MAX30100 pulse oximeter. Data is sent from an Arduino Nano to the ESP32 via serial communication.
- **Environmental Monitoring:** Tracks temperature and humidity using a DHT22 sensor.
- **Motion Detection:** Measures acceleration and angular velocity using an MPU6050 sensor.
- **Real-time Data Upload:** All sensor data is uploaded to Firebase Realtime Database, making it accessible from anywhere.

## Hardware Components

- **ESP32**: The main microcontroller responsible for data collection and transmission.
- **DHT22**: Measures temperature and humidity.
- **MPU6050**: Measures acceleration and gyroscopic data.
- **TinyGPS**: GPS module for location tracking.
- **MAX30100 Pulse Oximeter**: Measures heart rate and SpO2 levels.
- **Arduino Nano**: Collects heart rate and SpO2 data from the MAX30100 and sends it to the ESP32.
- **WiFi Network**: The ESP32 connects to a WiFi network to upload data to Firebase.

## Software Components

- **Arduino IDE**: Used to write and upload code to the ESP32 and Arduino Nano.
- **Firebase ESP32 Client Library**: For interacting with Firebase from the ESP32.
- **TinyGPS++ Library**: For parsing data from the GPS module.
- **DHT Sensor Library**: For reading data from the DHT22 sensor.
- **Adafruit MPU6050 Library**: For accessing data from the MPU6050 accelerometer and gyroscope.
- **Wire Library**: For I2C communication with the MPU6050 and MAX30100.

## Setup Instructions

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/abdulrafay517/iot-health-and-monitoring.git
   cd iot-health-and-monitoring
   ```

2. **Install Arduino Libraries:**
   - [Firebase ESP32 Client](https://github.com/mobizt/Firebase-ESP32)
   - [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)
   - [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)
   - [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
   - [MAX30100 Pulse Oximeter](https://github.com/oxullo/Arduino-MAX30100)
   
   Install these libraries via the Arduino IDE Library Manager.

3. **Configure Firebase:**
   - Create a Firebase project.
   - Add your ESP32 to Firebase.
   - Copy your API key and database URL and paste them into the code where indicated.

4. **Connect the Hardware:**
   - Follow the pin connections defined in the `Pin definitions` section of the code.
   - Ensure the WiFi credentials in the code match your network.

5. **Upload the Code:**
   - Open the project in Arduino IDE.
   - Select the appropriate board and port.
   - Upload the code to the ESP32 and Arduino Nano.

6. **Monitor Data:**
   - Use the Firebase Realtime Database console to monitor data in real-time.
   - Use the Serial Monitor in Arduino IDE to see the data being collected and sent.

## Usage

After setup, the system will automatically start collecting and uploading data to Firebase. The data can be accessed from the Firebase console or used in a custom application to visualize the data.

## Contributions

Contributions are welcome! Please open an issue or submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Adafruit** for the sensor libraries.
- **Firebase** for providing a robust backend for real-time data handling.
- **Arduino** for making microcontroller programming accessible.
