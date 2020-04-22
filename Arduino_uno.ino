/*
|**********************************************************************
* Project           : Laboratories of Industrial Electronics and Sensors
*                     Semestral project: Robot car
*
* Program name      : Arduino_uno.ino
*
* Author            : Vojtech Kozel (kozelvo1)
*
* Date created      : 20200420
*
* Purpose           : Gyro board code.
*
* Revision History  :
*
* Date        Author      Rev    Revision (Date in YYYYMMDD format) 
* 20200420    kozelvo1    1      Foundation of the project.
* 20200422    kozelvo1    2      Programming style changes.
*
|**********************************************************************
*/


#include <Wire.h>
#include <MPU6050_tockn.h>

#define Led_1 7
#define Led_2 8

MPU6050 mpu6050(Wire);

float actualAngle;
bool ledMode = true;

void setup() {
  /* Setup communication, processor pins and variables. */
  Serial.begin(9600);
  Wire.begin();  
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(Led_1, OUTPUT);
  pinMode(Led_2, OUTPUT);

  actualAngle = 0;
  Serial.print(1);
}

void loop() {
  /* Main loop. */

  // Gyro Z-ax angle.
  mpu6050.update();
  actualAngle = mpu6050.getAngleZ();
  Serial.print(actualAngle);

  // Hello blinking.
  if (ledMode) {
    digitalWrite(Led_2, HIGH);
    digitalWrite(Led_1, LOW);
  }
  else {
    digitalWrite(Led_1, HIGH);
    digitalWrite(Led_2, LOW);
  }
  ledMode = !ledMode;

  delay(100);
}
