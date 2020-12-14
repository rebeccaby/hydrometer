#include <Arduino_LSM9DS1.h>
#include <Arduino_HTS221.h>
//#include <ArduinoBLE.h>

float timeOld;
/*
// Randomly generated v4 uuid
BLEService hydrometerService("cdde75fc-0526-4a8d-96a2-d79cccf619e9");
BLEUnsignedCharCharacteristic hydrometerChar("cdde75fd-0526-4a8d-96a2-d79cccf619e9", BLERead | BLENotify);
*/

/*
float calculateTilt(float *gyro, float *acc) {
  
}
*/

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
  
  /*
  if(!BLE.begin()) {
    Serial.println("Failed to initialize BLE module!");
    while(1);
  }
  BLE.setLocalName("BluetoothHydrometer");
  BLE.setAdvertisedService(hydrometerService);
  hydrometerService.addCharacteristic(hydrometerChar);
  BLE.addService(hydrometerService);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
  */
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

void loop() {
  float acc_x, acc_y, acc_z, acc[3];
  float gyro_x, gyro_y, gyro_z, gyro[3];
  float tempC, tempF;

  // Fetch and print temperature values
  tempC = HTS.readTemperature();
  tempF = tempC * 1.8 + 32;
  printTempReadings(tempC, tempF);

  // Fetch and print accelerometer values
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    acc[0] = acc_x;
    acc[1] = acc_y;
    acc[2] = acc_z;
    printAccelReadings(acc_x, acc_y, acc_z);
  }

  // Fetch and print gyroscope values
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    gyro[0] = gyro_x;
    gyro[1] = gyro_y;
    gyro[2] = gyro_z;
    printGyroReadings(gyro_x, gyro_y, gyro_z);
  }

  //floatangle = tiltAngle(gyro, acc);
  
  /*
  Serial.print("LED service UUID = ");
  Serial.println(hydrometerService.uuid());
  */
  
  Serial.println();
  delay(1000);
}
