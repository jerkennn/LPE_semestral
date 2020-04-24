/*
|**********************************************************************
* Project           : Laboratories of Industrial Electronics and Sensors
*                     Semestral project: Robot car
*
* Program name      : Arduino_mega2560.ino
*
* Author            : Vojtech Kozel (kozelvo1)
*
* Date created      : 20200420
*
* Purpose           : Main board code.
*
* Revision History  :
*
* Date        Author      Rev    Revision (Date in YYYYMMDD format) 
* 20200420    kozelvo1    1      Foundation of the project.
* 20200421    kozelvo1    2      Basic form, PID controller implementation.
* 20200422    kozelvo1    3      Variables changed into structs.
*                                Added serial communication.
* 20200423    kozelvo1    4      Added bluetooth communication.
* 20200424    kozelvo1    5      Added RC mode.
*
|**********************************************************************
*/

#define BUFFER_SIZE 3
#define MANUAL_FLASHLIGHTS 52

// Reguator structures.
struct PID_parametres {
  float Kp;
  float Ki;
  float Kd;
};

struct regulator_parametres {
  struct PID_parametres constants;
  float previous_error;
  float integral;
  float setpoint;
  float regulator_out;
};

// Motors structures.
struct motor_parametres {
  unsigned char pin;
  unsigned char value;
};

struct motor {
  struct motor_parametres left;
  struct motor_parametres right;
};

// Serial communication structures.
struct bluetooth {
  char in_data[BUFFER_SIZE];
  char in_char;
};

struct gyro {
  char msg[10];
  bool initialized;
  bool initial_input;
  float angle;
  float prev_angle;
  float initialAngle;
};

struct serial_com {
  struct bluetooth bluetooth;
  struct gyro gyro;
};

struct order {
  char program_mode;
  char rc_order;
  char prev_rc_order;
  char prev_program_mode;
  char lights_mode;
};


struct regulator_parametres regulator = {0.1, 0.2, 0.3, 0, 0, 0, 0};
struct motor motor = {3, 255, 2, 255};
struct serial_com communication;
struct order order = {0, 0, 0, 0, 0};

float dt = 0.1;
  
void setup() {
  /* Setup communication, processor pins and variables. */
  Serial.begin(115200); // USB communication.
  Serial2.begin(9600); // Bluetooth communication.
  Serial3.begin(9600); // Gyro communication.

  pinMode(motor.left.pin, OUTPUT);
  pinMode(motor.right.pin, OUTPUT);
  
  communication.gyro.initialized = false;
  communication.gyro.initial_input = true;
}

float PID_controller(float input_angle);
void gyro_communication(); 
void bluetooth_communication();
  
void loop() {
  /* Main loop. */

  // Update program modes.
  bluetooth_communication();
  order.program_mode = communication.bluetooth.in_data[0];
  order.lights_mode = communication.bluetooth.in_data[1] - 48;
  order.rc_order= communication.bluetooth.in_data[2];

  // If needed update gyro offset.
  if ((order.program_mode && !order.prev_program_mode) || 
      (order.rc_order != order.prev_rc_order)) {
        communication.gyro.initial_input = true;
      }
  order.prev_program_mode = order.program_mode;
  order.prev_rc_order = order.rc_order;
  
  gyro_communication();
  
  float regulator_out = PID_controller(communication.gyro.angle);
  Serial.print(order.program_mode+10);
  Serial.print(" ");
  if (order.program_mode == 49) {
        Serial.print("****");
        Serial.print(" ");
        if (order.rc_order == 48) { // Forward.
           
           motor.left.value = 255;
           motor.right.value = (255-255); 
        }
        if (order.rc_order == 50) { // Left.
           motor.left.value = 0;
           motor.right.value = (255-255); 
        }
        if (order.rc_order == 49) { // Right.
           motor.left.value = 255;
           motor.right.value = (255-0); 
        }
  }
  else {
    motor.left.value = 0;
    motor.right.value = (255-0); 
  }
  analogWrite(motor.left.pin, motor.left.value);
  analogWrite(motor.right.pin, motor.right.value);

  if (order.lights_mode) {
    digitalWrite(MANUAL_FLASHLIGHTS, HIGH);
  }
  else {
    digitalWrite(MANUAL_FLASHLIGHTS, LOW);
  }
  
  Serial.print(communication.gyro.angle);
  Serial.print(" ");
  Serial.print(communication.bluetooth.in_data);
  Serial.println();
  

  delay(100);
}


float PID_controller(float input_angle) {
  /* PID controller.
   * :param input_angle: angle from gyro.
   * :return: regulator product. */
  
  float error = regulator.setpoint - input_angle;
  regulator.integral = regulator.integral + error*dt;
  float derivative = (error - regulator.previous_error)/dt;
  regulator.previous_error = error;

  return (regulator.constants.Kp*error + regulator.constants.Ki*
      regulator.integral + regulator.constants.Kd*derivative);
}


void gyro_communication() {
  /* Read angle values from gyro via serial communication. */

  while (Serial3.available() && !communication.gyro.initialized) {
    if (Serial3.read() == '#') {
      communication.gyro.initialized = true;
    }
  }
  
  int i=0;
  if (Serial3.available()) {
    while (Serial3.available() && i < 10 && communication.gyro.initialized) {
      char input = Serial3.read();
      if (input == '#') {
        break;
      }
      communication.gyro.msg[i++] = input;
    }
    
    float tmp = atof(communication.gyro.msg);

    // Offset.
    if (communication.gyro.initial_input) {
      communication.gyro.initialAngle = tmp;
      communication.gyro.prev_angle = tmp;
      communication.gyro.initial_input = false;
    }
    tmp -= communication.gyro.initialAngle;
    
    if (abs(tmp - communication.gyro.prev_angle) < 60) {
      communication.gyro.prev_angle = communication.gyro.angle;
      communication.gyro.angle = tmp;
    }
  }
}


void bluetooth_communication() {
  /* Read orders via serial bluetooth communication. */
  /* communication.bluetooth.in_data is array in form:
     [program_mode, lights_mode, rc_order]*/
  
  int i;
  byte bytes_cnt = Serial2.available(); // Number of bytes of input mesg.
  if (bytes_cnt) {
    int handle_bytes = bytes_cnt; // Number of handle bytes.
    int remaining_bytes=0; // Bytes burned off (buffer overrun).
    if (handle_bytes >= BUFFER_SIZE-1) {
      remaining_bytes = bytes_cnt - (BUFFER_SIZE-1); // Reduce bytes to buffer size.
    }
    if (handle_bytes == 3) { // filter
      for(i = 0; i < handle_bytes; i++) {
        communication.bluetooth.in_char = Serial2.read();
          communication.bluetooth.in_data[i] = communication.bluetooth.in_char;
      }
    communication.bluetooth.in_data[i]='\0';
    }
    for(i=0;i<remaining_bytes;i++) {
      Serial2.read(); // Get rid of remaining bytes.
    }
  }
}
