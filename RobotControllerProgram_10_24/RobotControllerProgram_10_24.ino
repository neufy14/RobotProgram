#include <SoftwareSerial.h>
#include <Stepper.h>
#include <math.h>
#include <MatrixMath.h>
#include <MemoryUsage.h>

int stepper_direction[5];
int stepper_steps[5];


void setup() {
  
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(300);
  //delay(100); 
  for (int i=0; i<10; i+=2) {
    stepper_direction[i] = i+3;
    stepper_steps[i] = i+4;
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(7, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  String message = "";
  const int row = 4;
  const int col = 4;
  String p_m = "";
  String velocity_direction = "";
  int step_max = 1000;
  int step_min = 90000;
  int step_get = 100;
  int step_fast = 0;
  int out = 0;
  int mover = 0;
  int io_counter = 0;
  int joint = 0;
  int steps_loc_start = 0;
  int steps_loc_stop = 0;
  int direction_loc_start = 0;
  int direction_loc_stop = 0;
  int x_pos = 0;
  int y_pos = 0;
  int z_pos = 0;
  int y_index = 0;
  int z_index = 0;
  int p_m_int = 0;
  double work_frame[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  }; //was float
  /*float input_matrix[6][4] = {
    {0, radians(-90), 41.694, 0},
    {0, 0, 0, 150},
    {0, radians(90), 0, 0},
    {0, radians(-90), -159.5, 0},
    {0, radians(90), 0, 0},
    {0, 0, -65, 0}
  };*/
  int start = 0;
  int ending = 0;
  int led_pin = 12;
  int complete_count = 0;
  double motor_positions[6] = {0,60,0,0,-60,0};
  double v_d_matrix[6] = {0,0,0,0,0,0};
  double* temp_positions;
    
  //Serial.println("Running");
  // put your main code here, to run repeatedly:
  //if (Serial.available() > 0) {
  message = Serial.readString();
  //Serial.print("Recieved!");
  //delayMicroseconds(75);
  //delay(10);
  if (message.startsWith("connect")) {
    //delay(10);
    Serial.print("Connected");
  }
  if (message.startsWith("Set IO")) {
    Serial.print("IO Ready");
  }
  if (message.startsWith("Joint")) {
  //while (out == 0) {
    //message = Serial.readString();
    joint = message.substring(4,5).toInt();
    steps_loc_start = message.indexOf("p", 7) + 1;
    steps_loc_stop = message.indexOf("D", steps_loc_start);
    direction_loc_start = message.indexOf("n", steps_loc_stop) + 1;
    stepper_steps[joint] = message.substring(steps_loc_start, steps_loc_stop).toInt();
    stepper_direction[joint] = message.substring(direction_loc_start).toInt();
    pinMode(stepper_steps[joint], OUTPUT);
    pinMode(stepper_direction[joint], OUTPUT);
    //Serial.print("Step");
    //Serial.print(stepper_steps[joint]);
    //Serial.print("Direction");
    //Serial.print(stepper_direction[joint]);
    Serial.print("Next");
    message = Serial.readString();   
    complete_count += 1;
    if (complete_count == 4) {
      complete_count = 0; 
    }
    
  }
  if (message.startsWith("Speed")) {
    step_get = message.substring(5,message.length()).toInt();
    Serial.print("Speed Set");
    step_fast = (step_min*(step_get/100));
  }
  /*if (message.startsWith("LMove") || message.startsWith("JMove")) {
    start = message.indexOf("J1")+2;
    ending = message.indexOf("J2")-1;
    reverse_angles[0] = message.substring(start, ending).toInt();
    start = message.indexOf("J2")+2;
    ending = message.indexOf("J3")-1;
    reverse_angles[1] = message.substring(start, ending).toInt();
    start = message.indexOf("J3")+2;
    ending = message.indexOf("J4")-1;
    reverse_angles[2] = message.substring(start, ending).toInt();
    start = message.indexOf("J4")+2;
    ending = message.indexOf("J5")-1;
    reverse_angles[3] = message.substring(start, ending).toInt();
    start = message.indexOf("J5")+2;
    reverse_angles[4] = message.substring(start).toInt();
    if (message.startsWith("JMove")) {
      for (int i=0; i<5; i++) {
        if (reverse_angles[i] != 0) {
          Serial.print(reverse_angles[i]);
          StepperMove(stepper_steps[i], stepper_direction[i], reverse_angles[i], step_fast);
        }
      }
    }
    Serial.print("Reached");      
  }*/
  if (message.startsWith("JMove")) {
    Serial.print("Spinning");
    p_m = message.substring(7);
    Serial.print(p_m);
    motor_positions[(message.substring(6).toInt()-1)] = StepperMove(stepper_steps[(message.substring(6).toInt()-1)], stepper_direction[(message.substring(6).toInt()-1)], p_m, step_min, motor_positions[(message.substring(6).toInt()-1)]);
  }
  if (message.startsWith("LMove")) {
    Serial.print("Spinning");
    p_m = message.substring(6,7);
    velocity_direction = message.substring(5,6);
    if (p_m == "+") {
      p_m_int = 1;
    }
    else if (p_m == "-") {
      p_m_int = -1;
    }
    if (velocity_direction == "x") {
      v_d_matrix[0] = 5 * p_m_int;
    }
    else if (velocity_direction == "y") {
      v_d_matrix[1] = 5 * p_m_int;
    }
    else if (velocity_direction == "z") {
      v_d_matrix[2] = 5 * p_m_int;
    }
    //Linear_Move((double*)motor_positions, 6, (double*)v_d_matrix, 6);

    temp_positions = Linear_Move(motor_positions, 6, v_d_matrix, 6);
    for (int i=0; i<sizeof(temp_positions); i++) {
      motor_positions[i] = temp_positions[i];
    }
  }
  if (message.startsWith("Running")) {
    Serial.print("Start");
    delay(5);
    message = Serial.readString();
    while (message.startsWith("x")) {
      y_index = message.indexOf("y", 0) - 1;
      z_index = message.indexOf("z", y_index) - 1;
      x_pos = message.substring(2, y_index).toInt();
      y_pos = message.substring(y_index+3, z_index).toInt();
      z_pos = message.substring(z_index+3).toInt();
      delay(5);
      Serial.print("Reached");
      message = Serial.readString();
    }
    Serial.print("Done");
  }
  if (message.startsWith("start")) {
    delay(5);
    Serial.print("s!");
    //StepperMove(stepper_steps[message[6]-1], stepper_direction[message[6]-1], message[7], step_fast);
  }
  //}
}

String ReadInput(){
  String message = "";
  if (Serial.available() > 0) {
    message = Serial.readString();
    //Serial.print(message);
  }
  /*if (Bluetooth.available() > 0) {
    message = Bluetooth.readString();  
  }*/

  return message;
}

int StepperMove(int step_pin, int direct_pin, String way, int fast, int joint_pos) {
  int go = 0;
  int steps = 0;
  int deg_moved = 0;
  int plus_minus = 0;
  String new_message = "";
  String first_part = "";
  String return_string = "";
  const int stepsPerRevolution = 200;
  if (way == "+") {
    digitalWrite(direct_pin, HIGH);
    first_part = "+";
    plus_minus = 1;
  }
  else if (way == "-") {
    digitalWrite(direct_pin, LOW);
    first_part = "-";
    plus_minus = -1;
  }
  while (new_message != "stop") {
    digitalWrite(7, HIGH);
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(fast);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(fast);
    steps += 1;
    if (Serial.available() > 0) {
      new_message = Serial.readString();
    }
  }
  digitalWrite(7, LOW);
  return_string = "stopping" + first_part + String(steps);
  Serial.print(return_string);
  first_part = "";
  joint_pos = (((steps / stepsPerRevolution) * 360) * plus_minus) + joint_pos;
  return joint_pos;
}

double** Forward_Kinematics(double joint_input[], int joint_input_len) {
  double a1 = 10;
  double a2 = 10;
  double a3 = 10;
  double a4 = 10;
  double a5 = 2;
  double a6 = 2;
  double input_matrix[6][4] = {{0, radians(90), 0, a1},
                   {0, 0, a2, 0},
                   {0, radians(+90), 0, 0},
                   {0, radians(-90), 0, (a3+a4)},
                   {0, radians(+90), 0, 0},
                   {0, 0, 0, (a5+a6)}};
  int work_frame[4][4];
  int tool_frame[4][4];
  double J1_matrix[4][4];
  double J2_matrix[4][4];
  double J3_matrix[4][4];
  double J4_matrix[4][4];
  double J5_matrix[4][4];
  double J6_matrix[4][4];
  double h_temp[4][4];
  double H0_6_Return[4][4];
  double** H0_2;
  double** H0_3;
  double** H0_4;
  double** H0_5;
  double** H0_6;
  
  /*float work_frame[4][4] = {
                  {1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}
                  };
  float tool_frame[4][4] = {
                  {1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}
                  };*/
  Serial.println("Joint Input");
  for (int i=0; i<joint_input_len; i++) {
    Serial.println(radians(joint_input[i]));
    input_matrix[i][0] = radians(joint_input[i]);
  }
  for (int i=0; i<4; i++) {
    tool_frame[i][i] = 1;
    work_frame[i][i] = 1;
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      if (i == 0) {
        if (j == 0) {
          J1_matrix[i][j] = cos(input_matrix[0][0]);
          J2_matrix[i][j] = cos(input_matrix[1][0]);
          J3_matrix[i][j] = cos(input_matrix[2][0]);
          J4_matrix[i][j] = cos(input_matrix[3][0]);
          J5_matrix[i][j] = cos(input_matrix[4][0]);
          J6_matrix[i][j] = cos(input_matrix[5][0]);
        }
        if (j == 1) {
            J1_matrix[i][j] = -1 * sin(input_matrix[0][0]) * cos(input_matrix[0][1]);
            J2_matrix[i][j] = -1 * sin(input_matrix[1][0]) * cos(input_matrix[1][1]);
            J3_matrix[i][j] = -1 * sin(input_matrix[2][0]) * cos(input_matrix[2][1]);
            J4_matrix[i][j] = -1 * sin(input_matrix[3][0]) * cos(input_matrix[3][1]);
            J5_matrix[i][j] = -1 * sin(input_matrix[4][0]) * cos(input_matrix[4][1]);
            J6_matrix[i][j] = -1 * sin(input_matrix[5][0]) * cos(input_matrix[5][1]);
        }
        if (j == 2) {
            J1_matrix[i][j] = sin(input_matrix[0][0]) * sin(input_matrix[0][1]);
            J2_matrix[i][j] = sin(input_matrix[1][0]) * sin(input_matrix[1][1]);
            J3_matrix[i][j] = sin(input_matrix[2][0]) * sin(input_matrix[2][1]);
            J4_matrix[i][j] = sin(input_matrix[3][0]) * sin(input_matrix[3][1]);
            J5_matrix[i][j] = sin(input_matrix[4][0]) * sin(input_matrix[4][1]);
            J6_matrix[i][j] = sin(input_matrix[5][0]) * sin(input_matrix[5][1]);
        }
        if (j == 3) {
            J1_matrix[i][j] = input_matrix[0][2]*cos(input_matrix[0][0]);
            J2_matrix[i][j] = input_matrix[1][2]*cos(input_matrix[1][0]);
            J3_matrix[i][j] = input_matrix[2][2]*cos(input_matrix[2][0]);
            J4_matrix[i][j] = input_matrix[3][2]*cos(input_matrix[3][0]);
            J5_matrix[i][j] = input_matrix[4][2]*cos(input_matrix[4][0]);
            J6_matrix[i][j] = input_matrix[5][2]*cos(input_matrix[5][0]);
        }
      }
      if (i == 1) {
        if (j == 0) {
          J1_matrix[i][j] = sin(input_matrix[0][0]);
          J2_matrix[i][j] = sin(input_matrix[1][0]);
          J3_matrix[i][j] = sin(input_matrix[2][0]);
          J4_matrix[i][j] = sin(input_matrix[3][0]);
          J5_matrix[i][j] = sin(input_matrix[4][0]);
          J6_matrix[i][j] = sin(input_matrix[5][0]);
        }
        if (j == 1) {
          J1_matrix[i][j] = cos(input_matrix[0][0]) * cos(input_matrix[0][1]);
          J2_matrix[i][j] = cos(input_matrix[1][0]) * cos(input_matrix[1][1]);
          J3_matrix[i][j] = cos(input_matrix[2][0]) * cos(input_matrix[2][1]);
          J4_matrix[i][j] = cos(input_matrix[3][0]) * cos(input_matrix[3][1]);
          J5_matrix[i][j] = cos(input_matrix[4][0]) * cos(input_matrix[4][1]);
          J6_matrix[i][j] = cos(input_matrix[5][0]) * cos(input_matrix[5][1]);
        }
        if (j == 2) {
          J1_matrix[i][j] = -1 * cos(input_matrix[0][0]) * sin(input_matrix[0][1]);
          J2_matrix[i][j] = -1 * cos(input_matrix[1][0]) * sin(input_matrix[1][1]);
          J3_matrix[i][j] = -1 * cos(input_matrix[2][0]) * sin(input_matrix[2][1]);
          J4_matrix[i][j] = -1 * cos(input_matrix[3][0]) * sin(input_matrix[3][1]);
          J5_matrix[i][j] = -1 * cos(input_matrix[4][0]) * sin(input_matrix[4][1]);
          J6_matrix[i][j] = -1 * cos(input_matrix[5][0]) * sin(input_matrix[5][1]);
        }
        if (j == 3) {
          J1_matrix[i][j] = input_matrix[0][2] * sin(input_matrix[0][0]);
          J2_matrix[i][j] = input_matrix[1][2] * sin(input_matrix[1][0]);
          J3_matrix[i][j] = input_matrix[2][2] * sin(input_matrix[2][0]);
          J4_matrix[i][j] = input_matrix[3][2] * sin(input_matrix[3][0]);
          J5_matrix[i][j] = input_matrix[4][2] * sin(input_matrix[4][0]);
          J6_matrix[i][j] = input_matrix[5][2] * sin(input_matrix[5][0]);
        }
      }
        if (i == 2) {
          if (j == 0) {
              J1_matrix[i][j] = 0;
              J2_matrix[i][j] = 0;
              J3_matrix[i][j] = 0;
              J4_matrix[i][j] = 0;
              J5_matrix[i][j] = 0;
              J6_matrix[i][j] = 0;
          }
          if (j == 1) {
            J1_matrix[i][j] = sin(input_matrix[0][1]);
            J2_matrix[i][j] = sin(input_matrix[1][1]);
            J3_matrix[i][j] = sin(input_matrix[2][1]);
            J4_matrix[i][j] = sin(input_matrix[3][1]);
            J5_matrix[i][j] = sin(input_matrix[4][1]);
            J6_matrix[i][j] = sin(input_matrix[5][1]);
          }
          if (j == 2) {
            J1_matrix[i][j] = cos(input_matrix[0][1]);
            J2_matrix[i][j] = cos(input_matrix[1][1]);
            J3_matrix[i][j] = cos(input_matrix[2][1]);
            J4_matrix[i][j] = cos(input_matrix[3][1]);
            J5_matrix[i][j] = cos(input_matrix[4][1]);
            J6_matrix[i][j] = cos(input_matrix[5][1]);
          }
          if (j == 3) {
            J1_matrix[i][j] = (input_matrix[0][3]);
            J2_matrix[i][j] = (input_matrix[1][3]);
            J3_matrix[i][j] = (input_matrix[2][3]);
            J4_matrix[i][j] = (input_matrix[3][3]);
            J5_matrix[i][j] = (input_matrix[4][3]);
            J6_matrix[i][j] = (input_matrix[5][3]);
          }
        }
        if (i == 3) {
          if (j == 0) {
            J1_matrix[i][j] = 0;
            J2_matrix[i][j] = 0;
            J3_matrix[i][j] = 0;
            J4_matrix[i][j] = 0;
            J5_matrix[i][j] = 0;
            J6_matrix[i][j] = 0;
          }
          if (j == 1) {
            J1_matrix[i][j] = 0;
            J2_matrix[i][j] = 0;
            J3_matrix[i][j] = 0;
            J4_matrix[i][j] = 0;
            J5_matrix[i][j] = 0;
            J6_matrix[i][j] = 0;
          }
          if (j == 2) {
            J1_matrix[i][j] = 0;
            J2_matrix[i][j] = 0;
            J3_matrix[i][j] = 0;
            J4_matrix[i][j] = 0;
            J5_matrix[i][j] = 0;
            J6_matrix[i][j] = 0;
          }
          if (j == 3) {
            J1_matrix[i][j] = 1;
            J2_matrix[i][j] = 1;
            J3_matrix[i][j] = 1;
            J4_matrix[i][j] = 1;
            J5_matrix[i][j] = 1;
            J6_matrix[i][j] = 1;
          }
        } 
    }
  }
  
  Serial.println("J1_matrix");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(J1_matrix[i][j]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  
  //Weird stuff happens with these Dot_Product functions
  //Serial.println("");
  //Serial.println("About to enter function");
  H0_2 = Dot_Product((double*)J1_matrix, (double*)J2_matrix, 4, 4, 4, 4, 1);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_2[i][j];
    }
  }
  
  Serial.println("H0_2");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(H0_2[i][j]);
      Serial.print("  ");
    }
    Serial.println("");
  }
  
  H0_3 = Dot_Product((double*)h_temp, (double*)J3_matrix, 4, 4, 4, 4, 1);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_3[i][j];
    }
  }
  H0_4 = Dot_Product((double*)h_temp, (double*)J4_matrix, 4, 4, 4, 4, 1); 
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_4[i][j];
    }
  }
  H0_5 = Dot_Product((double*)h_temp, (double*)J5_matrix, 4, 4, 4, 4, 1);
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      h_temp[i][j] = H0_5[i][j];
    }
  }
  H0_6 = Dot_Product((double*)h_temp, (double*)J6_matrix, 4, 4, 4, 4, 1);

  /*
  Serial.println("H0_6");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(H0_6[i][j]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  
  for (int i = 0; i < 4; i++) {
    delete[] H0_2[i];
    delete[] H0_3[i];
    delete[] H0_4[i];
    delete[] H0_5[i];
  }
  delete[] H0_2;
  delete[] H0_3;
  delete[] H0_4;
  delete[] H0_5;
  
  double** H0_6_pointer = new double*[4];
  for (int k = 0; k < 4; k++) {
    H0_6_pointer[k] = new double[4];
    for (int i = 0; i < 4; i++) {
      H0_6_pointer[k][i] = H0_6[k][i];
    }
  }
  for (int i = 0; i < 4; i++) {
    delete[] H0_6[i];
  }
  delete[] H0_6;
  /*for (int k = 0; k < (row); k++) {
    for (int i = 0; i < (col); i++) {
      for (int j = 0; j < (row); j++) {
        H0_6_pointer[k][i] = ((*(H0_6 + k * col + j)) * (*(H0_6 + j * col + i)));
      }
    }
  }*/
  
  return H0_6_pointer;
}

double* Jacobian(double joint_pos[], int joint_pos_len, double velocity[], int veloctiy_len) {
  double delta = 0.5; //degree change for calculating partial derivative;
  double jacobian_matrix[6][6];
  double jacobian_inv[6][6];
  double jacobian_inv_flip[6][6];
  double partial_derivative_matrix[4][4];
  double rotation_matrix_transpose[3][3];
  double rotation_matrix_derivative[3][3];
  double rotation_matrix_home[3][3];
  double joint_velocity_matrix[6];
  double** skew_symetric_matrix;
  double** temp;
  double** delta_matrix;
  double** home_matrix;
  double test1[4][4] = {{5,8,6,7},
                        {3,1,7,2},
                        {7,4,5,6},
                        {3,2,8,7}};
  double test2[4][4] = {{1,3,6,9},
                        {9,1,4,5},
                        {9,4,6,7},
                        {2,2,1,1}};

  //digitalWrite(LED_BUILTIN, HIGH);
  home_matrix = Forward_Kinematics(joint_pos, joint_pos_len);
  /*
  Serial.println("Joint Position");
  for (int i=0; i<joint_pos_len; i++) {
    Serial.println(joint_pos[i]);
  }
  Serial.println("Home Matrix");
  for (int i=0; i<4; i++) {
    for (int j=0; j<4; j++) {
      Serial.print(home_matrix[i][j]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < 6; i++) {
    joint_pos[i] = joint_pos[i] + delta;
    delta_matrix = Forward_Kinematics(joint_pos, joint_pos_len);
    /*
    Serial.println("Delta Matrix");
    for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
        Serial.print(delta_matrix[i][j]);
        Serial.print("   ");
      }
      Serial.println("");
    }
    */
    for (int j=0; j<4; j++) {
      for (int k=0; k<4; k++) {
        partial_derivative_matrix[j][k] = ((home_matrix[j][k] - delta_matrix[j][k]) / delta);
        //partial_derivative_matrix[j][k] = ((test1[j][k] - test2[j][k]) / delta);
        if (j < 3 || k < 3) {
          rotation_matrix_transpose[j][k] = home_matrix[k][j]; // rad
          rotation_matrix_home[j][k] = home_matrix[j][k]; // rad
          rotation_matrix_derivative[j][k] = partial_derivative_matrix[j][k]; //rad/rad
        }
      }
    }
    skew_symetric_matrix = Dot_Product((double*)rotation_matrix_derivative, (double*)rotation_matrix_transpose, 3, 3, 3, 3, 1);
    //Matrix.Multiply((double*)rotation_matrix_derivative, (double*)rotation_matrix_transpose, 3, 3, 3,(double*)skew_symetric_matrix);  
    jacobian_matrix[i][0] = partial_derivative_matrix[0][3]; //mm/rad
    jacobian_matrix[i][1] = partial_derivative_matrix[1][3]; //mm/rad
    jacobian_matrix[i][2] = partial_derivative_matrix[2][3]; //mm/rad
    jacobian_matrix[i][3] = skew_symetric_matrix[1][2]; //rad
    jacobian_matrix[i][4] = -1 * skew_symetric_matrix[0][2]; //rad
    jacobian_matrix[i][5] = skew_symetric_matrix[0][1]; //rad
    joint_pos[i] = joint_pos[i] - delta;
    //Serial.println("before I delete skew symetric");
    for (int i = 0; i < 4; i++) {
      delete[] delta_matrix[i];
      if (i < 3) {
        delete[] skew_symetric_matrix[i];
      }
    }
    delete[] delta_matrix;
    /*for (int i = 0; i < 3; i++) {
      delete[] skew_symetric_matrix[i];
    }*/
    delete[] skew_symetric_matrix;
  }
  Serial.println("Jacobian Post for loop");
  
  for (int i = 0; i < 4; i++) {
    
    delete[] home_matrix[i];
  }
  delete[] home_matrix;
  
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      jacobian_inv[i][j] = jacobian_matrix[i][j];
      //Serial.print(jacobian_inv[j][i]);V
      //Serial.print("   ");
    }
    //Serial.println("");
  }
  /*
  Serial.println("Velocity Matrix");
  for (int i=0; i<6; i++) {
    Serial.println(velocity[i]);
  }
  */
  Matrix.Invert((double*) jacobian_inv, 6);
  /*
  Serial.println("jacobian_inv");
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      Serial.print(jacobian_inv[j][i]);
      Serial.print("   ");
    }
    Serial.println("");
  }
  */
  Serial.println("Jacobian Inverse Flip");
  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      jacobian_inv_flip[j][i] = jacobian_inv[i][j];
      //Serial.print(jacobian_inv_flip[j][i]);
      //Serial.print("   ");
    }
    //Serial.println("");
  }
  
  temp = Dot_Product((double*)jacobian_inv_flip, (double*)velocity, 6, 6, 6, 1, 2);
  //Serial.println("temp[i][0]");
  for (int i=0; i<6; i++) {
    joint_velocity_matrix[i] = temp[0][i];
    //Serial.println(temp[i][0]);
    //Serial.println(temp[1][i]);
  }
  /*
  Serial.println("temp[i][1]");
  for (int i=0; i<6; i++) {
    Serial.println(temp[i][1]);
  }
  */
  /*for (int i = 0; i < 3; i++) {
    delete[] skew_symetric_matrix[i];
  }
  delete[] skew_symetric_matrix;*/
  /*
  for (int i = 0; i < 6; i++) {
    Serial.println("Delete for loop");
    delete[] temp[i];
    delete[] delta_matrix[i];
  }
  Serial.println("hello");
  for (int i = 0; i < 4; i++) {
    delete[] home_matrix[i];
    delete[] delta_matrix[i];
  }
  for (int i = 0; i < 3; i++) {
    delete[] skew_symetric_matrix[i];
  }
  delete[] skew_symetric_matrix;
  delete[] temp;
  delete[] home_matrix;
  delete[] delta_matrix;
  Serial.println("Passed delete");
  */
  Serial.print("sizeof(temp) = ");
  Serial.println(sizeof(temp));
  for (int k = 0; k < 2; k++) {
    Serial.println("before");
    delete temp[k];
    Serial.println("after");
  }
  delete[] temp;

  Serial.print("x position = ");
  Serial.println(home_matrix[0][3]);
  Serial.print("y position = ");
  Serial.println(home_matrix[1][3]);
  Serial.print("z position = ");
  Serial.println(home_matrix[2][3]);

  
  return joint_velocity_matrix;
}

double* Linear_Move (double joint_angles[], int joint_angle_len, double EOAT_velocity[], int EOAT_veloctiy_len) {
  double* j_v_matrix;
  String back_string = "stopping";
  int delta_t = 1;
  String linear_message = "";
  int times = 0;
  
  //while (linear_message != "stop") {
  while (linear_message.indexOf("stop") == -1) {
    //linear_message = "";
    Serial.println("Start of While Loop");
    Serial.print("Linear Message = ");
    Serial.println(linear_message);
    /*
    Serial.println("EOAT_Velocity");
    for (int i=0; i<6; i++) {
      Serial.println(EOAT_velocity[i]);
    }
    */
    //j_v_matrix = Jacobian(double joint_angles[], int joint_angle_len, double EOAT_velocity[], int EOAT_veloctiy_len);
    j_v_matrix = Jacobian(joint_angles, joint_angle_len, EOAT_velocity, EOAT_veloctiy_len);
    Serial.println("Post Jacobian");
    for (int i=0; i<sizeof(joint_angles); i++) {
      joint_angles[i] = joint_angles[i] + ((delta_t/1000) * j_v_matrix[i]);
    }
    delay(2);
    if (Serial.available() > 0) {
      Serial.println("Did I go in here???????????????????????????????????????????????????/?");
      linear_message = Serial.readString();
    }
    Serial.print("Times through loop = ");
    Serial.println(times);
    times++;
    FREERAM_PRINT;
    Serial.println("End of While Loop");
    delete[] j_v_matrix;
  }
  Serial.println("DID I BREAK THE WHILE LOOP?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?!?");
  //back_string = back_string + String(joint_angles[0]) + "A" + String(joint_angles[1]) + "B" + String(joint_angles[2]) + "C" + String(joint_angles[3]) + "D" + String(joint_angles[4]) + "E" + String(joint_angles[5]) + "F"; 
  Serial.print(back_string);
  return joint_angles; 
}

double** Dot_Product (double* matrix_1, double* matrix_2, int row_1, int col_1, int row_2, int col_2, int type) {
  double** matrix_solution = new double*[col_2];
  if (type == 1) {
    for (int k = 0; k < row_2; k++) {
      matrix_solution[k] = new double[col_1];
      // the second time calling this function we get stuck in this loop
      for (int ii = 0; ii < row_2; ii++) {
        //Serial.print("i = ");
        //Serial.println(ii);
        matrix_solution[k][ii] = 0;
      }
      //Serial.println("BREAK");
      //digitalWrite(LED_BUILTIN, LOW);
    }
    for (int i = 0; i < row_1; i++) {
      for (int j = 0; j < col_2; j++) {
        for (int k = 0; k < row_2; k++) {
          matrix_solution[i][j] = ((*(matrix_1 + i * col_1 + k)) * (*(matrix_2 + k * row_1 + j))) + matrix_solution[i][j];
        }
      }
    }
  }
  else if (type == 2) {
    for (int k = 0; k < row_2; k++) {
      matrix_solution[k] = new double[col_2];
      for (int i = 0; i < (col_2+1); i++) {
        matrix_solution[k][i] = 0;
      }
    }
    for (int i = 0; i < row_2; i++) {
      for (int j = 0; j < row_2; j++) {
        //for (int k = 0; k < row_2; k++) {
        matrix_solution[i][0] = ((*(matrix_1 + i * col_1 + j)) * (*(matrix_2 + j * col_2 + 0))) + matrix_solution[i][0];
        //}
      }
    }
  }

  return matrix_solution;
}


/*int doSomething(x, y, z) {
  
  return 
}*/
