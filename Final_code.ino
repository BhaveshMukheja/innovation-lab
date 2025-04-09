#include <Wire.h>
#include <Servo.h>

const int MPU = 0x68; // MPU6050 I2C address

// Servo objects and pins
Servo servoRoll;  // Controls roll (X-axis)
Servo servoPitch; // Controls pitch (Y-axis)
Servo servoYaw;   // Controls yaw (Z-axis)
const int servoRollPin = 8;
const int servoPitchPin = 10;
const int servoYawPin = 12;

// IMU variables
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// Servo calibration
const int servoMin = 0;    // Minimum servo angle (degrees)
const int servoMax = 180;   // Maximum servo angle (degrees)
const int servoCenter = 90; // Center position (degrees)

// Stabilization parameters
const float maxTiltAngle = 30.0; // Maximum tilt angle to compensate for (degrees)
const float stabilizationGain = 1.0; // How aggressively to counteract movement (0.0 to 1.0)

void setup() {
  Serial.begin(19200);
  
  // Attach servos to pins
  servoRoll.attach(servoRollPin);
  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);
  
  // Center all servos
  servoRoll.write(servoCenter);
  servoPitch.write(servoCenter);
  servoYaw.write(servoCenter);
  delay(1000); // Give servos time to reach center position

  // Initialize MPU6050
  Wire.begin();                      
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);                  
  Wire.write(0x00);                 
  Wire.endTransmission(true);        
  
  // Calculate IMU errors
  calculate_IMU_error();
  delay(20);
  
  // Initialize timing variables
  previousTime = millis();
}

void loop() {
  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; 

  // Calculate Roll and Pitch from accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope data === //
  currentTime = millis();  
  elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  previousTime = currentTime;

  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Subtract the bias error
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  // Integrate gyroscope data to get angle change
  gyroAngleX += GyroX * elapsedTime; 
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  // Complementary filter
  float alpha = 0.96;
  roll = alpha * gyroAngleX + (1 - alpha) * accAngleX;
  pitch = alpha * gyroAngleY + (1 - alpha) * accAngleY;

  // Print values
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" | Pitch: "); Serial.print(pitch);
  Serial.print(" | Yaw: "); Serial.println(yaw);

  // Control servos to counteract movement
  controlServos();
}

void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    AccErrorX += (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    AccErrorY += (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    c++;
  }
  AccErrorX /= 200;
  AccErrorY /= 200;
  
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    GyroErrorX += GyroX / 131.0;
    GyroErrorY += GyroY / 131.0;
    GyroErrorZ += GyroZ / 131.0;
    c++;
  }
  GyroErrorX /= 200;
  GyroErrorY /= 200;
  GyroErrorZ /= 200;

  Serial.println("IMU Calibration Complete:");
  Serial.print("AccErrorX: "); Serial.println(AccErrorX);
  Serial.print("AccErrorY: "); Serial.println(AccErrorY);
  Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: "); Serial.println(GyroErrorZ);
}

void controlServos() {
  // Calculate servo positions (opposite of IMU movement)
  int rollServoPos = servoCenter - constrain(roll * stabilizationGain, -maxTiltAngle, maxTiltAngle) * (servoMax - servoCenter) / maxTiltAngle;
  int pitchServoPos = servoCenter - constrain(pitch * stabilizationGain, -maxTiltAngle, maxTiltAngle) * (servoMax - servoCenter) / maxTiltAngle;
  int yawServoPos = servoCenter - constrain(yaw * stabilizationGain, -maxTiltAngle, maxTiltAngle) * (servoMax - servoCenter) / maxTiltAngle;

  // Constrain to servo limits
  rollServoPos = constrain(rollServoPos, servoMin, servoMax);
  pitchServoPos = constrain(pitchServoPos, servoMin, servoMax);
  yawServoPos = constrain(yawServoPos, servoMin, servoMax);

  // Move servos
  servoRoll.write(rollServoPos);
  servoPitch.write(pitchServoPos);
  servoYaw.write(yawServoPos);

  // Optional: Print servo positions
  /*
  Serial.print("Servo Roll: "); Serial.print(rollServoPos);
  Serial.print(" | Servo Pitch: "); Serial.print(pitchServoPos);
  Serial.print(" | Servo Yaw: "); Serial.println(yawServoPos);
  */
}