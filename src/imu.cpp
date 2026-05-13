#define SDA_PIN 21 
#define SCL_PIN 22
#include <Wire.h>

#include <SparkFun_BNO080_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void imu_setup()
{
  Serial.println();
  Serial.println("BNO080 Read Example");


  //Are you using a ESP? Check this issue for more information: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/16
//  //=================================
 delay(100); //  Wait for BNO to boot
//  // Start i2c and BNO080
 Wire.flush();   // Reset I2C
  if (!myIMU.begin(BNO080_DEFAULT_ADDRESS, Wire)) delay(1000);
  Wire.begin(21,22);
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
//  //=================================

  myIMU.enableGyro(50); //Send data update every 50ms




    Serial.println(F("Gyro enabled"));
  Serial.println(F("Output in form x, y, z, in radians per second"));
}

void imu_loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getGyroX();
    float y = myIMU.getGyroY();
    float z = myIMU.getGyroZ();

    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);
    Serial.print(F(","));

    Serial.println();
  }
}

