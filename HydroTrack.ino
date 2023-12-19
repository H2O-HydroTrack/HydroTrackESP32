#include "BluetoothSerial.h"
#include <ArduinoJson.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PIN_TRIGGER 18
#define PIN_ECHO 19
#define TRANSMISSION_INTERVAL 2500

const int SENSOR_MAX_RANGE = 300;  // in cm
unsigned long duration;
unsigned int distance;

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

class BottleData {
public:
  float X;
  float Y;
  float Z;
  unsigned int distance;

  BottleData()
    : X(0), Y(0), Z(0) {}
};

void setup() {
  SerialBT.begin("ESP32test");
  Serial.begin(9600);

  initUltrasonic();
  initGyro();

  delay(1000);
}

void loop() {
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);

  duration = pulseIn(PIN_ECHO, HIGH);
  distance = duration / 58;

  if (distance > SENSOR_MAX_RANGE || distance <= 0) {
    distance = 0;
  } else {
    Serial.println("Distance to object: " + String(distance) + " cm");
  }

  // Get new sensor events with the reading
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  BottleData bottleData;
  bottleData.X = a.acceleration.x;
  bottleData.Y = a.acceleration.y;
  bottleData.Z = a.acceleration.z;

  if (SerialBT.available()) {
    //inputFromOtherSide = SerialBT.readString();
    StaticJsonDocument<1024> data;
    data["x"] = bottleData.X;
    data["y"] = bottleData.Y;
    data["z"] = bottleData.Z;
    data["distance"] = distance;

    String output;
    serializeJsonPretty(data, output);
    SerialBT.println(output);
  }

  delay(TRANSMISSION_INTERVAL);
}

void initUltrasonic() {
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void initGyro() {
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
}
