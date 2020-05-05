# LPE_semestral
Arduino robot car

## WIRING: Arduino UNO 
Gyro MPU6050 i2c (TWI): SDA, SCL \
Led1: digital pin 7 \
Led2: digital pin 8 \
Serial communication with Arduino MEGA 2560: serial port no. 1 (Rx digital pin 0, Tx digital pin 1) 

## WIRING: Arduino MEGA 2560 
Manual flashlights control: digital pin 52 \
Left IR sensor (black line detection): digital pin 51 \
Right IR sensor (black line detection): digital pin 50 \
motor.left.pin1: 2 \
motor.left.pin2: 3 \
motor.right.pin1: 4 \
motor.right.pin2: 5 \
Serial communication with Arduino UNO: serial port no. 2 (Rx digital pin 17, Tx digital pin 16) \
Serial communication with bluetooth (HC-06): serial port no. 3 (Rx digital pin 15, Tx digital pin 14)

## Desktop app
### Keys:
Up ~ forward \
Down ~ back \
Left ~ left \
Right ~ right \
Space ~ stop \

![Alt text](app_window.png?raw=true "Title")
