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
* 20200429    kozelvo1    6      Setup driving regulator.
* 20200430    kozelvo1    7      Revision of code.
*                                Added line regulator.
*
|**********************************************************************
*/


#define BUFFER_SIZE 3
#define MANUAL_FLASHLIGHTS 52
#define IR_LEFT 51
#define IR_RIGHT 50

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
};

// Motors structures.
struct motor_parametres {
  unsigned char pin1;
  unsigned char pin2;
  float value1;
  float value2;
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


struct regulator_parametres driving_regulator = {3, 2.5, 1, 0, 0, 0};
struct regulator_parametres line_regulator = {1, 0, 0, 0, 0, 0};
struct motor motor = {2, 3, 255, 0, 4, 5, 255, 0};
struct serial_com communication;
struct order order = {0, 3, 3, 0, 0};

float dt = 0.1;
  
void setup() {
  /* Setup communication, processor pins and variables. */
  Serial.begin(9600); // USB communication.
  Serial2.begin(9600); // Gyro communication.
  Serial3.begin(9600); // Bluetooth communication.

  pinMode(motor.left.pin1, OUTPUT);
  pinMode(motor.right.pin1, OUTPUT);
  pinMode(motor.left.pin2, OUTPUT);
  pinMode(motor.right.pin2, OUTPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  communication.gyro.initialized = false;
  communication.gyro.initial_input = true;
}

float driving_PID_controller(float input_angle);
float line_PI_controller(int line_error);
void gyro_communication(); 
void bluetooth_communication();
  
void loop() {
  /* Main loop. */

  // Check for "line situation".
  bool left_ir = digitalRead(IR_LEFT);
  bool right_ir = digitalRead(IR_RIGHT);
  bool init_line_forward = false;
  if (!left_ir && !right_ir) inti_line_forward = true;

  // Update program modes.
  bluetooth_communication();
  order.program_mode = communication.bluetooth.in_data[0] - 48;
  order.lights_mode = communication.bluetooth.in_data[1] - 48;
  order.rc_order= communication.bluetooth.in_data[2] - 48;

  // Init new direction.
  if ((order.program_mode == 1 && order.prev_program_mode != 1) || 
      (order.rc_order != order.prev_rc_order)
      (!init_line_forward)) {
        communication.gyro.initial_input = true;
        driving_regulator.integral = 0;
        driving_regulator.previous_error = 0; 
      }
  // Init manual control.
  if (order.program_mode == 1 && order.prev_program_mode != 1) {
    motor.left.value1 = 0;
    motor.left.value2 = 0;
    motor.right.value1 = 0;
    motor.right.value2 = 0;
    order.rc_order = 3;
  }
  
  order.prev_program_mode = order.program_mode;
  order.prev_rc_order = order.rc_order;
  
  gyro_communication();
  
  float driving_regulator_out = driving_PID_controller(communication.gyro.angle);

  if (order.program_mode == 1) {
        if (order.rc_order == 0) { // Forward.
          motor.right.value1 = 255;
          float tmp = motor.right.value1;
          tmp += driving_regulator_out;
          tmp = round(tmp);
          if (tmp > 255) tmp = 255;
          if (tmp < 0) tmp = 0;
          motor.left.value1 = 255;  motor.left.value2 = 0;
          motor.right.value1 = tmp; motor.right.value2 = 0;
        }
        if (order.rc_order == 2) { // Left.
           motor.left.value1 = 255; motor.left.value2 = 0;
           motor.right.value1 = 0;  motor.right.value2 = 255; 
        }
        if (order.rc_order == 1) { // Right.
           motor.left.value1 = 0;    motor.left.value2 = 255;
           motor.right.value1 = 255; motor.right.value2 = 0;
        }
        if (order.rc_order == 3) { // Stop.
            motor.left.value1 = 0;  motor.left.value2 = 0;
            motor.right.value1 = 0; motor.right.value2 = 0;
        }
        if (order.rc_order == 4) { // Back.
          motor.right.value2 = 255;
          float tmp = motor.right.value2;
          //tmp -= driving_regulator_out;
          tmp = round(tmp); 
          if (tmp > 255) tmp = 255;
          if (tmp < 0) tmp = 0;
          motor.left.value1 = 0;  motor.left.value2 = 255;
          motor.right.value1 = 0; motor.right.value2 = tmp;
        }
  }
  else {
    motor.left.value1 = 255;  motor.left.value2 = 0;
    motor.right.value1 = 255; motor.right.value2 = 0;
    if (letf_sensor and !right_sensor) { // Left sensor is above a line.
      motor.left.value1 -= line_PI_controller(1);
    }
    else if (!letf_sensor and right_sensor) { // Right sensor is above a line.
      motor.right.value1 -= line_PI_controller(1);
    }
    else
    {
      line_regulator.integral = 0;
      motor.right.value1 = 255;
      float tmp = motor.right.value1;
      tmp += driving_regulator_out;
      tmp = round(tmp);
      if (tmp > 255) tmp = 255;
      if (tmp < 0) tmp = 0;
      motor.right.value1 = tmp;
    }
    
  }
  analogWrite(motor.left.pin1,  motor.left.value1);
  analogWrite(motor.left.pin2,  motor.left.value2);
  analogWrite(motor.right.pin1, motor.right.value1);
  analogWrite(motor.right.pin2, motor.right.value2);

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


float line_PI_controller(int error) {
  /* driving PID controller.
   * :param (int) error: type of situation.
   * :return: regulator product. */
  
  line_regulator.integral += error;
  return (line_regulator.constants.Kp*error) + 
         (line_regulator.constants.Ki*line_regulator.integral);
}


float driving_PID_controller(float input_angle) {
  /* driving PID controller.
   * :param input_angle: angle from gyro.
   * :return: regulator product. */
  
  float error = driving_regulator.setpoint - input_angle;
  driving_regulator.integral = driving_regulator.integral + error*dt;
  float derivative = (error - driving_regulator.previous_error)/dt;
  driving_regulator.previous_error = error;

  return (driving_regulator.constants.Kp*error + driving_regulator.constants.Ki*
          driving_regulator.integral + driving_regulator.constants.Kd*derivative);
}


void gyro_communication() {
  /* Read angle values from gyro via serial communication. */
  while (Serial2.available() && !communication.gyro.initialized) {
    if (Serial2.read() == '#') {
      communication.gyro.initialized = true;
    }
  }
  
  int i=0;
  if (Serial2.available()) {
    while (Serial2.available() && i < 10 && communication.gyro.initialized) {
      char input = Serial2.read();
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
  byte bytes_cnt = Serial3.available(); // Number of bytes of input mesg.
  if (bytes_cnt) {
    int handle_bytes = bytes_cnt; // Number of handle bytes.
    int remaining_bytes=0; // Bytes burned off (buffer overrun).
    if (handle_bytes >= BUFFER_SIZE-1) {
      remaining_bytes = bytes_cnt - (BUFFER_SIZE-1); // Reduce bytes to buffer size.
    }
    if (handle_bytes == 3) { // filter
      for(i = 0; i < handle_bytes; i++) {
        communication.bluetooth.in_char = Serial3.read();
          communication.bluetooth.in_data[i] = communication.bluetooth.in_char;
      }
    communication.bluetooth.in_data[i]='\0';
    }
    for(i=0;i<remaining_bytes;i++) {
      Serial3.read(); // Get rid of remaining bytes.
    }
  }
}
