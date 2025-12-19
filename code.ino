#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// ---------------- DHT11 Configuration ----------------
#define DHTPIN 2          // DHT11 data pin connected to digital pin 2
#define DHTTYPE DHT11     // Type of DHT sensor

DHT dht(DHTPIN, DHTTYPE);

// ---------------- Sensor Objects ----------------
Adafruit_BMP280 bmp;      // BMP280 sensor object
MPU6050 mpu;              // MPU6050 sensor object

// ---------------- GPS Configuration ----------------
SoftwareSerial gpsSerial(4, 3); // GPS RX -> pin 4, TX -> pin 3
TinyGPSPlus gps;

// ---------------- Variables for Motion Data ----------------
float ax, ay, az;  // Acceleration values
float gx, gy, gz;  // Gyroscope values

void setup() {

  // Start serial communication for monitoring data
  Serial.begin(9600);

  // Start GPS communication
  gpsSerial.begin(9600);

  // Initialize I2C communication
  Wire.begin();

  // Initialize DHT11 sensor
  dht.begin();

  // Initialize BMP280 sensor
  if (!bmp.begin(0x76)) {
    Serial.println("Error: BMP280 sensor not detected!");
    while (1); // Stop execution if sensor is not found
  }

  // Initialize MPU6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 sensor not connected!");
    while (1); // Stop execution if sensor is not found
  }

  Serial.println("Multiple Sensor Telemetry System Started");
  Serial.println("----------------------------------------");
}

void loop() {

  // ----------- Read DHT11 Data -----------
  float humidity = dht.readHumidity();          // Read humidity
  float tempDHT = dht.readTemperature();        // Read temperature

  // ----------- Read BMP280 Data -----------
  float tempBMP = bmp.readTemperature();        // Temperature in °C
  float pressure = bmp.readPressure() / 100.0; // Pressure in hPa
  float altitude = bmp.readAltitude(1013.25);  // Altitude in meters

  // ----------- Read MPU6050 Data -----------
  mpu.getAcceleration(&ax, &ay, &az);  // Acceleration values
  mpu.getRotation(&gx, &gy, &gz);      // Gyroscope values

  // ----------- Read GPS Data -----------
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // ----------- Display Logged Data -----------
  Serial.println("Telemetry Data:");

  Serial.print("DHT11 Temperature (°C): ");
  Serial.print(tempDHT);
  Serial.print(" | Humidity (%): ");
  Serial.println(humidity);

  Serial.print("BMP280 Temperature (°C): ");
  Serial.print(tempBMP);
  Serial.print(" | Pressure (hPa): ");
  Serial.print(pressure);
  Serial.print(" | Altitude (m): ");
  Serial.println(altitude);

  Serial.print("Acceleration (X, Y, Z): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);

  Serial.print("Gyroscope (X, Y, Z): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);

  // Check if GPS data is valid
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("GPS Status: Waiting for satellite signal...");
  }

  Serial.println("----------------------------------------");

  // Delay before next reading
  delay(2000);
}