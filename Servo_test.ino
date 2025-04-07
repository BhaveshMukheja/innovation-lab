#include <Wire.h>
#include <Servo.h>

// MPU-6050 I2C Address
const int MPU = 0x68;

// Servo Objects
Servo servo1, servo2, servo3;

// IMU Variables
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float roll, pitch;

void setup() {
  Serial.begin(9600);
  
  // Attach Servos to PWM Pins (Change if needed)
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  
  // Initialize MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up MPU-6050
  Wire.endTransmission(true);
  
  Serial.println("Circuit Test Started");
}

void loop() {
  // === Test Servos (Sweep 0° to 180°) ===
  Serial.println("Testing Servo 1 (Pin 9)");
  for (int pos = 0; pos <= 180; pos++) {
    servo1.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    servo1.write(pos);
    delay(15);
  }

  Serial.println("Testing Servo 2 (Pin 10)");
  for (int pos = 0; pos <= 180; pos++) {
    servo2.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    servo2.write(pos);
    delay(15);
  }

  Serial.println("Testing Servo 3 (Pin 11)");
  for (int pos = 0; pos <= 180; pos++) {
    servo3.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos--) {
    servo3.write(pos);
    delay(15);
  }

  // === Test MPU-6050 ===
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 bytes (Accelerometer)
  
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // Raw to G-force
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  // Calculate Roll/Pitch (Degrees)
  roll = atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2)) * 180 / PI;
  pitch = atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2)) * 180 / PI;

  // Print IMU Data
  Serial.print("MPU-6050 | Roll: "); Serial.print(roll);
  Serial.print("° | Pitch: "); Serial.print(pitch);
  Serial.println("°");

  delay(1000); // Pause between tests
}