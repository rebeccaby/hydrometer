#include <Arduino_LSM9DS1.h>
#include <Arduino_HTS221.h>
//#include <ArduinoBLE.h>

void printTempReadings(float tempC, float tempF);
void printAccelReadings(float x, float y, float z);
void printGyroReadings(float x, float y, float z);
void calculateTilt(float gyro[3], float acc[3]);

#define GYRO_SENS 245.0
#define M_PI 3.14159265359

unsigned long time_old, time_delta;
float pitch = 0, roll = 0, yaw = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Started");

  if(!IMU.begin()) {
    Serial.println("Failed to initialize IMU Sensors!");
    while(1);
  }

  if(!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while(1);
  }

  time_old = millis();
}

// theta = angular velocity * change in time
// use complementary filter for the low-pass filter for accel data
// integrate filtered accel data with gyro data to get more accurate tilt result
void calculateTilt(float gyro[3], float acc[3]) {
  
  float pitch_Acc, roll_acc, yaw_acc;

  pitch = -atan2f(acc[0], acc[2]) * 180 / M_PI;
  roll = -atan2f(acc[2], acc[1]) * 180 / M_PI;
  yaw = -atan2f(acc[1], acc[2]) * 180 / M_PI;

  /*
  pitch_acc = 
  roll_acc = 
  yaw_acc = 
  */

  Serial.println("WITHOUT FILTER -------------");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\tRoll = ");
  Serial.print(roll);
  Serial.print("\tYaw = ");
  Serial.println(yaw);

  /*
  pitch = (gyro[0] / GYRO_SENS) * time_delta;
  roll = (gyro[1] / GYRO_SENS) * time_delta;
  yaw = (gyro[2] / GYRO_SENS) * time_delta;

  Serial.println("WITH SENSITIVITY");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\tRoll = ");
  Serial.print(roll);
  Serial.print("\tYaw = ");
  Serial.println(yaw);
  */
  
  delay(1000);
}

void loop() {
  float acc_x, acc_y, acc_z, acc[3];
  float gyro_x, gyro_y, gyro_z, gyro[3];
  float tempC, tempF;
  
  time_delta = millis() - time_old;

  // Fetch and print temperature values
  tempC = HTS.readTemperature();
  tempF = tempC * 1.8 + 32;
  //printTempReadings(tempC, tempF);

  // Fetch and print accelerometer values
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    acc[0] = acc_x;
    acc[1] = acc_y;
    acc[2] = acc_z;
    //printAccelReadings(acc_x, acc_y, acc_z);
  }

  // Fetch and print gyroscope values
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    gyro[0] = gyro_x;
    gyro[1] = gyro_y;
    gyro[2] = gyro_z;
    //printGyroReadings(gyro_x, gyro_y, gyro_z);
  }
  
  calculateTilt(gyro, acc);
  time_old += time_delta;

  Serial.println();
  delay(1000);
}

void printTempReadings(float tempC, float tempF) {
  Serial.println("TEMPERATURE ------------------------");
  Serial.print(tempC);
  Serial.print("°C = ");
  Serial.print(tempF);
  Serial.print("°F");
  Serial.println('\n');
}

void printAccelReadings(float x, float y, float z) {
  Serial.println("ACCELEROMETER ----------------------");
  Serial.print("x = ");
  Serial.print(x);
  Serial.print(",\ty = ");
  Serial.print(y);
  Serial.print(",\tz = ");
  Serial.print(z);
  Serial.println('\n');
}

void printGyroReadings(float x, float y, float z) {
  Serial.println("GYROSCOPE --------------------------");
  Serial.print("x = ");
  Serial.print(x);
  Serial.print(",\ty = ");
  Serial.print(y);
  Serial.print(",\tz = ");
  Serial.print(z);
  Serial.println('\n');
}
