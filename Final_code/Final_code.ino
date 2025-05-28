#define BLYNK_TEMPLATE_ID "add_yours"
#define BLYNK_TEMPLATE_NAME "3dadGAS"
#define BLYNK_AUTH_TOKEN "add_yours"

#include <Arduino.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_BMP085.h>

// Wi-Fi credentials
char ssid[] = "TE-Data";
char pass[] = "************";

// OLED Display (128x64)
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);

// BMP180 Sensor
Adafruit_BMP085 bmp;

// RPM Measurement Variables
#define IR_SENSOR_PIN D4  // Moved to D4 to avoid I2C conflict
volatile unsigned long rpmtime = 0;
float rpmfloat = 0;
unsigned int rpm = 0;
const unsigned int bladeCount = 8;
volatile unsigned long lastPulseTime = 0;
Ticker rpmTimeoutChecker;

// Sensor Data Averaging
const int sampleCount = 5;
float pressureSamples[sampleCount] = {0};
float tempSamples[sampleCount] = {0};
int sampleIndex = 0;

// Constants & Variables for Calculations
const float gasConstant = 287.05;      // J/(kg·K)
const float pipeDiameter = 0.158;      // meters
const float area = 3.14159 * (pipeDiameter / 2) * (pipeDiameter / 2);
const float calibrationFactor = 0.0023;   // Fixed Calibration Factor (enha)
float massFlowRate = 0;

// Interrupt Service Routine (ISR) for RPM
void IRAM_ATTR RPM_ISR() {
    unsigned long currentTime = micros();

    if (lastPulseTime > 0) {  
        unsigned long delta = currentTime - lastPulseTime; 

        if (abs((long)delta - (long)rpmtime) > 500) {  
            rpmtime = delta;
        }
    }
    
    lastPulseTime = currentTime;
}

// Function to calculate RPM
void checkRPMTimeout() {
    if (millis() - (lastPulseTime / 1000) > 1500) { 
        rpm = 0;
    } else if (rpmtime > 0) {
        rpmfloat = (60000000.0 / (rpmtime * bladeCount)); 
        rpm = (unsigned int)rpmfloat;
    }
}

// Read Pressure and Temperature from BMP180
void readSensors(float &pressure, float &temperature) {
    pressure = bmp.readPressure();  // Keep pressure in Pascals
    temperature = bmp.readTemperature();
}

// Compute Averaged Sensor Values
void computeAverages(float &pressure, float &temperature) {
    float totalPressure = 0, totalTemp = 0;
    for (int i = 0; i < sampleCount; i++) {
        totalPressure += pressureSamples[i];
        totalTemp += tempSamples[i];
    }
    pressure = totalPressure / sampleCount;
    temperature = totalTemp / sampleCount;
}

void setup() {
    Serial.begin(115200);
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r); 

    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), RPM_ISR, FALLING);
    rpmTimeoutChecker.attach(1, checkRPMTimeout);

    if (!bmp.begin()) {
        Serial.println("BMP180 sensor not found!"); //you didn't connect the sensor
        while (1);
    }

    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
    Blynk.run();

    float pressure, temperature;
    readSensors(pressure, temperature);

    pressureSamples[sampleIndex] = pressure;
    tempSamples[sampleIndex] = temperature;
    sampleIndex = (sampleIndex + 1) % sampleCount;

    computeAverages(pressure, temperature);

    float tempK = temperature + 273.15;
    float airDensity = pressure / (gasConstant * tempK);
    float volumeFlowRate = area * calibrationFactor * rpm; // Q=Av where V-> linear velocity of fluid = K*rpm of the turbine
    massFlowRate = volumeFlowRate * airDensity; // m dot= Q*rho

    // Send data to Blynk
    Blynk.virtualWrite(V1, rpm);
    Blynk.virtualWrite(V3, pressure);
    Blynk.virtualWrite(V4, temperature);
    Blynk.virtualWrite(V6, volumeFlowRate * 3600);
    Blynk.virtualWrite(V7, massFlowRate);


    Serial.print("RPM: "); Serial.print(rpm);
    Serial.print(" | Pressure: "); Serial.print(pressure); Serial.print(" Pa");
    Serial.print(" | Temperature: "); Serial.print(temperature); Serial.println(" °C");

    // Display Mass Flow Rate, Temperature, and Pressure on OLED
    u8x8.clear();
    u8x8.setCursor(0, 0);
    u8x8.print("Mass Flow:");
    u8x8.setCursor(0, 1);
    u8x8.print(massFlowRate, 3); // 3 decimal places 
    u8x8.print(" kg/s");

    u8x8.setCursor(0, 3);
    u8x8.print("Temp:");
    u8x8.setCursor(0, 4);
    u8x8.print(temperature, 1);
    u8x8.print(" C");

    u8x8.setCursor(0, 6);
    u8x8.print("Pressure:");
    u8x8.setCursor(0, 7);
    u8x8.print(pressure);
    u8x8.print(" Pa");

    delay(333);
}
