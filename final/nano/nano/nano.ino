#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000
#define SENSOR_RESET_INTERVAL_MS 60000  // Reset sensor every minute

PulseOximeter pox;
uint32_t tsLastReport = 0;
uint32_t lastBeatTime = 0;

uint32_t lastSensorReset = 0;

float lastValidHeartRate = 0;
float lastValidSpO2 = 0;

void onBeatDetected() {
    lastBeatTime = millis();
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    initializeSensor();
}

void initializeSensor() {
    Serial.println("Initializing pulse oximeter..");
    if (!pox.begin()) {
        Serial.println("FAILED");
        delay(1000);
        return;  // Return to loop and try again
    } else {
        Serial.println("SUCCESS");
        pox.setOnBeatDetectedCallback(onBeatDetected);
        pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    }
}

void loop() {
    uint32_t currentTime = millis();
    
    if (currentTime - lastSensorReset > SENSOR_RESET_INTERVAL_MS) {
        initializeSensor();
        lastSensorReset = currentTime;
    }

    pox.update();

    if (currentTime - tsLastReport > REPORTING_PERIOD_MS) {
        float heartRate = pox.getHeartRate();
        float spo2 = pox.getSpO2();

        if (heartRate > 0 && spo2 > 0 && currentTime - lastBeatTime < 5000) {
            lastValidHeartRate = heartRate;
            lastValidSpO2 = spo2;
        }
        
        // Always send data, even if it's the last valid reading
        Serial.print(lastValidHeartRate, 1);
        Serial.print(",");
        Serial.println(lastValidSpO2, 1);

        tsLastReport = currentTime;
    }

    delay(10);
}