#include <Wire.h>
#include <Servo.h> // Include the Servo library

const int MPU = 0x68; // MPU6050 I2C address

// Servo objects for roll, pitch, and yaw
Servo rollServo;
Servo pitchServo;
Servo yawServo;

// Variables to hold raw sensor data
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

// Variables for calculated angles
float accAngleX, accAngleY;
float gyroAngleX, gyroAngleY;

// Roll, pitch, and yaw outputs
float roll, pitch, yaw;

// Error variables
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

// Time variables for gyroscope integration
float elapsedTime, currentTime, previousTime;

void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize I2C communication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050
  Wire.write(0x6B);                  // Access the power management register
  Wire.write(0x00);                  // Wake up the MPU6050
  Wire.endTransmission(true);        // End communication

  // Attach servos to respective pins
  rollServo.attach(9);  // Roll servo on pin 9
  pitchServo.attach(10); // Yaw servo on pin 10
  yawServo.attach(8);  // pitch servo on pin 8

  // Calculate IMU error values for calibration
  calculate_IMU_error();
  delay(50);
}

void loop() {
  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);                  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);    // Read 6 registers total
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  // Calculate accelerometer angles
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope data === //
  previousTime = currentTime;        // Store previous time
  currentTime = millis();            // Get current time
  elapsedTime = (currentTime - previousTime) / 1000; // Calculate elapsed time in seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43);                  // Start with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);    // Read 6 registers total
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // Convert to deg/s
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Subtract gyroscope error
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  // Integrate gyroscope data
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;

  // === Apply complementary filter === //
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // === Map and constrain angles for servos === //
  int rollServoPos = map(roll, 90, -90, 0, 180);
  int pitchServoPos = map(pitch, -180, 180, 0, 180);
  int yawServoPos = map(yaw, 90, -90, 0, 180);

  // Constrain values to servo range
  rollServoPos = constrain(rollServoPos, 0, 180);
  pitchServoPos = constrain(pitchServoPos, 0, 180);
  yawServoPos = constrain(yawServoPos, 0, 180);

  // Move servos to calculated positions
  rollServo.write(rollServoPos);
  pitchServo.write(pitchServoPos);
  yawServo.write(yawServoPos);

  // === Print angles to serial monitor === //
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(rollServoPos);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Yaw: ");
  Serial.println(yaw);

  delay(50); // Short delay for stability
}

void calculate_IMU_error() {
  // Variables for summing readings
  float sumAccX = 0, sumAccY = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  // Read accelerometer and gyroscope 200 times
  for (int i = 0; i < 200; i++) {
    // Read accelerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    // Sum accelerometer angles
    sumAccX += (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    sumAccY += (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);

    // Read gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    // Sum gyroscope values
    sumGyroX += GyroX / 131.0;
    sumGyroY += GyroY / 131.0;
    sumGyroZ += GyroZ / 131.0;

    delay(50);
  }

  // Calculate average errors
  AccErrorX = sumAccX / 200;
  AccErrorY = sumAccY / 200;
  GyroErrorX = sumGyroX / 200;
  GyroErrorY = sumGyroY / 200;
  GyroErrorZ = sumGyroZ / 200;

  // Print error values
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
