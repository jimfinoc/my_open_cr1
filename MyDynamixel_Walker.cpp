#include "MyDynamixel_Walker.h"

MyDynamixelWalker::MyDynamixelWalker() {
  #if defined DEBUG
  Serial.begin(115200);
  #endif
}

// MyDynamixelWalker::~MyDynamixelWalker() {
//
// }
void MyDynamixelWalker::initializeRobot(uint8_t number_of_motors,uint8_t *dxl_motor_ids){
  initializeDynamixels();
  _number_of_motors = number_of_motors;
  for (uint8_t i = 0; i < number_of_motors; i++) {
    _dxl_motor_ids[i] = dxl_motor_ids[i];
  }
  // zeroFootPosition();
  configureTrotFeetPositions();
  configureWalkFeetPositions();
}

void MyDynamixelWalker::configureTrotFeetPositions(){
  // initializeDynamixels();
  _step = 0;

  _initial_foot_placement_x[FL] =  _initial_foot_placement_x[FR] =  0.15;
  _initial_foot_placement_x[BR] =  _initial_foot_placement_x[BL] = -0.15; //in meters

  _initial_foot_placement_y[FL] =  _initial_foot_placement_y[BL] =  0.15;
  _initial_foot_placement_y[FR] =  _initial_foot_placement_y[BR] = -0.15;

  // update the moving z positions
  _moving_z[FR][3] = _moving_z[FR][4] = _moving_z[FR][5] = _up_leg;
  _moving_z[FR][0] = _moving_z[FR][1] = _moving_z[FR][2] = _moving_z[FR][6] = _moving_z[FR][7] = _moving_z[FR][8] = 0.0;

  _moving_z[BL][3] = _moving_z[BL][4] = _moving_z[BL][5] = _up_leg;
  _moving_z[BL][0] = _moving_z[BL][1] = _moving_z[BL][2] = _moving_z[BL][6] = _moving_z[BL][7] = _moving_z[BL][8] = 0.0;

  _moving_z[FL][7] = _moving_z[FL][8] = _moving_z[FL][1] = _up_leg;
  _moving_z[FL][0] = _moving_z[FL][2] = _moving_z[FL][3] = _moving_z[FL][4] = _moving_z[FL][5] = _moving_z[FL][6] = 0.0;

  _moving_z[BR][7] = _moving_z[BR][8] = _moving_z[BR][1] = _up_leg;
  _moving_z[BR][0] = _moving_z[BR][2] = _moving_z[BR][3] = _moving_z[BR][4] = _moving_z[BR][5] = _moving_z[BR][6] = 0.0;

  // update the moving x positions
  _moving_x[FR][1] = _moving_x[BL][1] = -_step_x_distance/4;
  _moving_x[FR][2] = _moving_x[BL][2] = -_step_x_distance/2;
  _moving_x[FR][3] = _moving_x[BL][3] = -_step_x_distance/4;
  _moving_x[FR][5] = _moving_x[BL][5] =  _step_x_distance/4;
  _moving_x[FR][6] = _moving_x[BL][6] =  _step_x_distance/2;
  _moving_x[FR][7] = _moving_x[BL][7] =  _step_x_distance/4;

  _moving_x[BR][1] = _moving_x[FL][1] =  _step_x_distance/4;
  _moving_x[BR][2] = _moving_x[FL][2] =  _step_x_distance/2;
  _moving_x[BR][3] = _moving_x[FL][3] =  _step_x_distance/4;
  _moving_x[BR][5] = _moving_x[FL][5] = -_step_x_distance/4;
  _moving_x[BR][6] = _moving_x[FL][6] = -_step_x_distance/2;
  _moving_x[BR][7] = _moving_x[FL][7] = -_step_x_distance/4;

  _moving_x[FR][0] = _moving_x[BL][0] = _moving_x[BR][0] = _moving_x[FL][0] = 0.0;
  _moving_x[FR][4] = _moving_x[BL][4] = _moving_x[BR][4] = _moving_x[FL][4] = 0.0;
  _moving_x[FR][8] = _moving_x[BL][8] = _moving_x[BR][8] = _moving_x[FL][8] = 0.0;

  // update the moving y positions
  _moving_y[FR][1] = _moving_y[BL][1] = -_step_y_distance/4;
  _moving_y[FR][2] = _moving_y[BL][2] = -_step_y_distance/2;
  _moving_y[FR][3] = _moving_y[BL][3] = -_step_y_distance/4;
  _moving_y[FR][5] = _moving_y[BL][5] =  _step_y_distance/4;
  _moving_y[FR][6] = _moving_y[BL][6] =  _step_y_distance/2;
  _moving_y[FR][7] = _moving_y[BL][7] =  _step_y_distance/4;

  _moving_y[BR][1] = _moving_y[FL][1] =  _step_y_distance/4;
  _moving_y[BR][2] = _moving_y[FL][2] =  _step_y_distance/2;
  _moving_y[BR][3] = _moving_y[FL][3] =  _step_y_distance/4;
  _moving_y[BR][5] = _moving_y[FL][5] = -_step_y_distance/4;
  _moving_y[BR][6] = _moving_y[FL][6] = -_step_y_distance/2;
  _moving_y[BR][7] = _moving_y[FL][7] = -_step_y_distance/4;

  _moving_y[FR][0] = _moving_y[BL][0] = _moving_y[BR][0] = _moving_y[FL][0] = 0.0;
  _moving_y[FR][4] = _moving_y[BL][4] = _moving_y[BR][4] = _moving_y[FL][4] = 0.0;
  _moving_y[FR][8] = _moving_y[BL][8] = _moving_y[BR][8] = _moving_y[FL][8] = 0.0;

  // update the turning x and y positions FL
  _turning_x[FL][0] = 0.0;
  _turning_x[FL][4] = 0.0;
  _turning_x[FL][8] = 0.0;
  _turning_x[FL][1] = _turning_x[FL][3] = hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * cos( _step_rotation/4*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_x[FL][2] =                     hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * cos( _step_rotation/2*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_x[FL][5] = _turning_x[FL][7] = hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * cos(-_step_rotation/4*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_x[FL][6] =                     hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * cos(-_step_rotation/2*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_y[FL][0] = 0.0;
  _turning_y[FL][4] = 0.0;
  _turning_y[FL][8] = 0.0;
  _turning_y[FL][1] = _turning_y[FL][3] = hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * sin( _step_rotation/4*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_y[FL][2] =                     hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * sin( _step_rotation/2*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_y[FL][5] = _turning_y[FL][7] = hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * sin(-_step_rotation/4*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];
  _turning_y[FL][6] =                     hypotf(_initial_foot_placement_x[FL],_initial_foot_placement_y[FL]) * sin(-_step_rotation/2*PI/180+_leg_rotation[FL]) - _initial_foot_placement_x[FL];

  // update the turning x and y positions FR
  _turning_x[FR][0] = 0.0;
  _turning_x[FR][4] = 0.0;
  _turning_x[FR][8] = 0.0;
  _turning_x[FR][1] = _turning_x[FR][3] = hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * cos(-_step_rotation/4*PI/180+_leg_rotation[FR]) - _initial_foot_placement_x[FR];
  _turning_x[FR][2] =                     hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * cos(-_step_rotation/2*PI/180+_leg_rotation[FR]) - _initial_foot_placement_x[FR];
  _turning_x[FR][5] = _turning_x[FR][7] = hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * cos( _step_rotation/4*PI/180+_leg_rotation[FR]) - _initial_foot_placement_x[FR];
  _turning_x[FR][6] =                     hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * cos( _step_rotation/2*PI/180+_leg_rotation[FR]) - _initial_foot_placement_x[FR];
  _turning_y[FR][0] = 0.0;
  _turning_y[FR][8] = 0.0;
  _turning_y[FR][4] = 0.0;
  _turning_y[FR][1] = _turning_y[FR][3] = hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * sin(-_step_rotation/4*PI/180+_leg_rotation[FR]) - _initial_foot_placement_y[FR];
  _turning_y[FR][2] =                     hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * sin(-_step_rotation/2*PI/180+_leg_rotation[FR]) - _initial_foot_placement_y[FR];
  _turning_y[FR][5] = _turning_y[FR][7] = hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * sin( _step_rotation/4*PI/180+_leg_rotation[FR]) - _initial_foot_placement_y[FR];
  _turning_y[FR][6] =                     hypotf(_initial_foot_placement_x[FR],_initial_foot_placement_y[FR]) * sin( _step_rotation/2*PI/180+_leg_rotation[FR]) - _initial_foot_placement_y[FR];

  // update the turning x and y positions BL
  _turning_x[BL][0] = 0.0;
  _turning_x[BL][4] = 0.0;
  _turning_x[BL][8] = 0.0;
  _turning_x[BL][1] = _turning_x[BL][3] = hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * cos(-_step_rotation/4*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_x[BL][2] =                     hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * cos(-_step_rotation/2*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_x[BL][5] = _turning_x[BL][7] = hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * cos( _step_rotation/4*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_x[BL][6] =                     hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * cos( _step_rotation/2*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_y[BL][0] = 0.0;
  _turning_y[BL][4] = 0.0;
  _turning_y[BL][8] = 0.0;
  _turning_y[BL][1] = _turning_y[BL][3] = hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * sin(-_step_rotation/4*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_y[BL][2] =                     hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * sin(-_step_rotation/2*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_y[BL][5] = _turning_y[BL][7] = hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * sin( _step_rotation/4*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];
  _turning_y[BL][6] =                     hypotf(_initial_foot_placement_x[BL],_initial_foot_placement_y[BL]) * sin( _step_rotation/2*PI/180+_leg_rotation[BL]) - _initial_foot_placement_x[BL];

  // update the turning x and y positions BR
  _turning_x[BR][0] = 0.0;
  _turning_x[BR][4] = 0.0;
  _turning_x[BR][8] = 0.0;
  _turning_x[BR][1] = _turning_x[BR][3] = hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * cos( _step_rotation/4*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_x[BR][2] =                     hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * cos( _step_rotation/2*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_x[BR][5] = _turning_x[BR][7] = hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * cos(-_step_rotation/4*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_x[BR][6] =                     hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * cos(-_step_rotation/2*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_y[BR][0] = 0.0;
  _turning_y[BR][4] = 0.0;
  _turning_y[BR][8] = 0.0;
  _turning_y[BR][1] = _turning_y[BR][3] = hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * sin( _step_rotation/4*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_y[BR][2] =                     hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * sin( _step_rotation/2*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_y[BR][5] = _turning_y[BR][7] = hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * sin(-_step_rotation/4*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];
  _turning_y[BR][6] =                     hypotf(_initial_foot_placement_x[BR],_initial_foot_placement_y[BR]) * sin(-_step_rotation/2*PI/180+_leg_rotation[BR]) - _initial_foot_placement_x[BR];

  // for (int i = 0; i < _number_of_motors; i++) {
  //   uint8_t dxl_error;
  //   _dxl_motor_ids[i] = _dxl_motor_ids[i];
  //   setMotorVelocity(_dxl_motor_ids[i], 50, dxl_error);
  //   delay(5);
  // }
  // zeroFootPosition();
  // zeroGunAiming();
  // // _dxl_motor_ids[14] = _dxl_motor_ids[14];
}
void MyDynamixelWalker::calculateTrotAngles(){
  #if defined DEBUG
  Serial.println("");
  Serial.println("-calculateTrotAngles");
  Serial.print("currentMFB:");
  Serial.print(currentMFB);
  Serial.print("_lastMFB:");
  Serial.print(_lastMFB);
  Serial.println("");
  Serial.print("currentMLR:");
  Serial.print(currentMLR);
  Serial.print("_lastMLR:");
  Serial.print(_lastMLR);
  Serial.println("");
  Serial.print("currentTLR:");
  Serial.print(currentTLR);
  Serial.print("_lastTLR:");
  Serial.print(_lastTLR);
  Serial.println("");
  Serial.print("_step:");
  Serial.print(_step);
  Serial.println("");
  Serial.println("");
  #endif
  // Serial.println("=Moving Data:");
  for (int leg = 0; leg < 4; leg++) {
    // Serial.print("leg:");
    // Serial.print(leg);
    // Serial.println("");
    for (int step = 0; step < 9; step++) {
      // Serial.print("step:");
      // Serial.print(step);
      // Serial.print(" ");
      // Serial.print(" _moving_x:");
      // Serial.print(_moving_x[leg][step]);
      // Serial.print(" _moving_y:");
      // Serial.print(_moving_y[leg][step]);
      // Serial.print(" _moving_z:");
      // Serial.print(_moving_z[leg][step]);
      // Serial.print(" _turning_x:");
      // Serial.print(_turning_x[leg][step]);
      // Serial.print(" _turning_y:");
      // Serial.print(_turning_y[leg][step]);
      // Serial.println("");
    }
  }

  float total_x, total_y, total_z;
  float angle1, angle2, angle3;
  float s, s_squared,c_squared,cos_angle,sin_angle;

  for (int leg = 0; leg < 4; leg++) {
    total_x = _initial_foot_placement_x[leg] + _lastMFB * _moving_x[leg][_step] + _lastTLR * _turning_x[leg][_step];
    total_y = _initial_foot_placement_y[leg] + _lastMLR * _moving_y[leg][_step] + _lastTLR * _turning_y[leg][_step];
    total_z = _initial_foot_placement_z + _moving_z[leg][_step] ;
    #if defined DEBUG
    Serial.print("leg ");
    Serial.print(leg);
    Serial.print(" x y z ");
    Serial.print(total_x);
    Serial.print(" ");
    Serial.print(total_y);
    Serial.print(" ");
    Serial.print(total_z);
    Serial.println();
    #endif

    angle1 = __angle_wrap(atan2(total_y - _joint_placement_y[leg], total_x - _joint_placement_x[leg]) - _leg_rotation[leg]);
    _next_angle_values[leg*3+0] = angle1;

    s = hypotf(total_y - _joint_placement_y[leg], total_x - _joint_placement_x[leg]);
    s_squared = s*s;
    c_squared = s_squared + pow(_initial_foot_placement_z +_moving_z[leg][_step] - _joint_placement_z,2);
    cos_angle = ( c_squared - powf(_link_a,2) - powf(_link_b,2) ) / (-2.0 * _link_a *_link_b);
    if (cos_angle > 1) {cos_angle = 1;}
    if (cos_angle < -1) {cos_angle = -1;}

    sin_angle = sqrtf(1-powf(cos_angle,2));

    #if defined DEBUG
    Serial.println("s s_squared c_squared cos_angle sin_angle");
    Serial.print(s,3);
    Serial.print(" ");
    Serial.print(s_squared,3);
    Serial.print(" ");
    Serial.print(c_squared,4);
    Serial.print(" ");
    Serial.print(cos_angle,2);
    Serial.print(" ");
    Serial.print(sin_angle,2);
    Serial.println();
    #endif
    angle3 = __angle_wrap( PI - atan2(sin_angle,cos_angle) );
    _next_angle_values[leg*3+2] = angle3;

    angle2 =  __angle_wrap( -PI/2 + atan2(_initial_foot_placement_z - _joint_placement_z, s) + atan2(_link_b*sin(angle3),_link_a + _link_b*cos(angle3)) );
    _next_angle_values[leg*3+1] = angle2;
    #if defined DEBUG
    Serial.println("");
    Serial.print("leg angles");
    Serial.print(leg);
    Serial.print(":");
    Serial.print(angle1/PI*180,9);
    Serial.print(" ");
    Serial.print(angle2/PI*180,9);
    Serial.print(" ");
    Serial.print(angle3/PI*180,9);
    Serial.println();
    #endif
  }
  // Serial.println();
  #if defined DEBUG
  Serial.println();

  Serial.print("_next_angle_values:");
  #endif
  for (int i = 0; i < 12; i++) {
    if (i%3==0) {
      #if defined DEBUG
      Serial.println("");
      #endif
    }
    // Serial.print(i);
    // Serial.print(" ");
    #if defined DEBUG
    Serial.print(_next_angle_values[i]);
    // _next_motor_positions[i] = int(_next_angle_values[i]/PI*180/300*1023)+512;
    Serial.print(" ");
    Serial.print(int(_next_angle_values[i]/PI*180/300*1023)+512);
    Serial.print(" ");
    #endif
  }
  #if defined DEBUG
  Serial.println(" ");
  #endif
}

void MyDynamixelWalker::configureWalkFeetPositions(){
  _step = 0;
  _initial_foot_placement_x[FL] =  _initial_foot_placement_x[FR] =  0.15;
  _initial_foot_placement_x[BR] =  _initial_foot_placement_x[BL] = -0.15; //in meters

  _initial_foot_placement_y[FL] =  _initial_foot_placement_y[BL] =  0.15;
  _initial_foot_placement_y[FR] =  _initial_foot_placement_y[BR] = -0.15;

}
void MyDynamixelWalker::calculateWalkAngles(){
  #if defined DEBUG
  Serial.println("");
  Serial.println("-calculateWalkAngles");
  Serial.print("currentMFB:");
  Serial.print(currentMFB);
  Serial.print("_lastMFB:");
  Serial.print(_lastMFB);
  Serial.println("");
  Serial.print("currentMLR:");
  Serial.print(currentMLR);
  Serial.print("_lastMLR:");
  Serial.print(_lastMLR);
  Serial.println("");
  Serial.print("currentTLR:");
  Serial.print(currentTLR);
  Serial.print("_lastTLR:");
  Serial.print(_lastTLR);
  Serial.println("");
  Serial.println("");
  #endif

  float total_x, total_y, total_z;
  float angle1, angle2, angle3;
  float s, s_squared,c_squared,cos_angle,sin_angle;
  float turning_walk_x = 0;
  float turning_walk_y = 0;

  for (int leg = 0; leg < 4; leg++) {
    if (_step > 8) {

      total_x = _initial_foot_placement_x[leg] * 1.2;
      total_y = _initial_foot_placement_y[leg] * 1.2;
      total_z = _initial_foot_placement_z + _up_leg;
    }
    else if ((leg == FR and _step == 1) or (leg == FL and _step == 5) or (leg == BL and _step == 3) or (leg == BR and _step == 7)) {
      total_x = _initial_foot_placement_x[leg];
      total_y = _initial_foot_placement_y[leg];
      total_z = _initial_foot_placement_z + _up_leg;
      if (currentMoving == 1)
        _foot_Zeroed[leg] = 0;
    }
    else if ((leg == FR and _step == 2) or (leg == FL and _step == 6) or (leg == BL and _step == 4) or (leg == BR and _step == 8) or (_step == 0))  {
      total_x = _initial_foot_placement_x[leg] + _lastMFB * _step_x_distance/2;
      total_y = _initial_foot_placement_y[leg] + _lastMLR * _step_y_distance/2;
      total_z = _initial_foot_placement_z;
      _foot_Zeroed[leg] = 1;
    }
    else {
      if (currentMoving == 1)
        _foot_Zeroed[leg] = 0;
      if (_lastTLR != 0) {
        float degrees_turning = _lastTLR * _step_rotation/2;
        float pre_turn_rotation = atan2(_foot_position_y[leg],_foot_position_x[leg]);
        turning_walk_x =  hypotf(_foot_position_x[leg],_foot_position_y[leg]) * cos(degrees_turning/180*PI + pre_turn_rotation) - _foot_position_x[leg];
        turning_walk_y =  hypotf(_foot_position_x[leg],_foot_position_y[leg]) * sin(degrees_turning/180*PI + pre_turn_rotation) - _foot_position_y[leg];
      }
      total_x = _foot_position_x[leg] - _lastMFB * _step_x_distance/6 - turning_walk_x;
      total_y = _foot_position_y[leg] - _lastMLR * _step_y_distance/6 - turning_walk_y;
      // total_z = _foot_position_z[leg];
      total_z = _initial_foot_placement_z;
      // _foot_position_z[leg] = _initial_foot_placement_z;
    }
    _foot_position_x[leg] = total_x;
    _foot_position_y[leg] = total_y;
    _foot_position_z[leg] = total_z;
    // total_x = _initial_foot_placement_x[leg] + _lastMFB * _moving_x[leg][_step] + _lastTLR * _turning_x[leg][_step];
    // total_y = _initial_foot_placement_y[leg] + _lastMLR * _moving_y[leg][_step] + _lastTLR * _turning_y[leg][_step];
    // total_z = _initial_foot_placement_z + _moving_z[leg][_step] ;

    angle1 = __angle_wrap(atan2(total_y - _joint_placement_y[leg], total_x - _joint_placement_x[leg]) - _leg_rotation[leg]);
    _next_angle_values[leg*3+0] = angle1;

    s = hypotf(total_y - _joint_placement_y[leg], total_x - _joint_placement_x[leg]);
    s_squared = s*s;
    c_squared = s_squared + pow(_foot_position_z[leg] - _joint_placement_z,2);
    cos_angle = ( c_squared - powf(_link_a,2) - powf(_link_b,2) ) / (-2.0 * _link_a *_link_b);
    if (cos_angle > 1) {cos_angle = 1;}
    if (cos_angle < -1) {cos_angle = -1;}

    sin_angle = sqrtf(1-powf(cos_angle,2));

    #if defined DEBUG
    // Serial.println("s s_squared c_squared cos_angle sin_angle");
    // Serial.print(s,3);
    // Serial.print(" ");
    // Serial.print(s_squared,3);
    // Serial.print(" ");
    // Serial.print(c_squared,4);
    // Serial.print(" ");
    // Serial.print(cos_angle,2);
    // Serial.print(" ");
    // Serial.print(sin_angle,2);
    // Serial.println();
    #endif
    angle3 = __angle_wrap( PI - atan2(sin_angle,cos_angle) );
    _next_angle_values[leg*3+2] = angle3;

    angle2 =  __angle_wrap( -PI/2 + atan2(_initial_foot_placement_z - _joint_placement_z, s) + atan2(_link_b*sin(angle3),_link_a + _link_b*cos(angle3)) );
    _next_angle_values[leg*3+1] = angle2;
    #if defined DEBUG
    Serial.print("leg ");
    Serial.print(leg);
    Serial.print(" turning_walk_x");
    Serial.print(turning_walk_x);
    Serial.print(" turning_walk_y");
    Serial.print(turning_walk_y);
    Serial.print(" movingX");
    Serial.print(_lastMFB * _step_x_distance/2/6);
    Serial.print(" movingY");
    Serial.print(_lastMLR * _step_y_distance/2/6);
    Serial.println();
    Serial.print("leg ");
    Serial.print(leg);
    Serial.print(" x y z ");
    Serial.print(total_x);
    Serial.print(" ");
    Serial.print(total_y);
    Serial.print(" ");
    Serial.print(total_z);
    Serial.println();
    // Serial.println("");
    // Serial.print("leg angles");
    // Serial.print(leg);
    // Serial.print(":");
    // Serial.print(angle1/PI*180,9);
    // Serial.print(" ");
    // Serial.print(angle2/PI*180,9);
    // Serial.print(" ");
    // Serial.print(angle3/PI*180,9);
    // Serial.println();
    #endif

  }
  // Serial.println();
  #if defined DEBUG
  Serial.println();
  Serial.print("_next_angle_values:");
  for (int i = 0; i < 12; i++) {
    if (i%3==0) Serial.println("");
    // Serial.print(i);
    // Serial.print(" ");
    Serial.print(_next_angle_values[i]);
    // _next_motor_positions[i] = int(_next_angle_values[i]/PI*180/300*1023)+512;
    Serial.print(" ");
    Serial.print(int(_next_angle_values[i]/PI*180/300*1023)+512);
    Serial.print(" ");
  }
  Serial.println(" ");
  #endif
}

void MyDynamixelWalker::setLoaderSpeed(uint16_t loader_speed){
  uint8_t dxl_error = 0;                          // Dynamixel error
  loaderSpeed = loader_speed;
  setMotorVelocity(73, loader_speed, dxl_error);
}

void MyDynamixelWalker::reportLastMotorPosition(uint16_t *motor_position) {
  int number_of_motors = 14;
  for (int i = 0; i < number_of_motors; i++) {
    motor_position[i] = _last_motor_positions[i];
  }
}
void MyDynamixelWalker::reportNextMotorPositionFloats(float *motor_position) {
  int number_of_motors = 14;
  for (int i = 0; i < number_of_motors; i++) {
    // must convert 0-1023 to radian position centered on zero
    //after converting to degrees, convert to radians
    motor_position[i] = float(_next_motor_positions[i])*0.29/180.0*PI-PI ;
  }
}
void MyDynamixelWalker::reportNextMotorPosition(uint16_t *motor_position) {
  int number_of_motors = 14;
  for (int i = 0; i < number_of_motors; i++) {
    // must convert 0-1023 to radian position centered on zero
    //after converting to degrees, convert to radians
    motor_position[i] = _next_motor_positions[i];
  }
  motor_position[14] = 0;
}

void MyDynamixelWalker::reportMotorVelocitiesFloats(float *motor_velocity) {
  int number_of_motors = 15;
  for (int i = 0; i < number_of_motors; i++) {
    // must convert 0-1023 to radian per second
    // after converting to RPM, convert to radians per second
    motor_velocity[i] = float(_motor_velocities[i]) / 9 / 60 * 2 * PI;
  }
}
void MyDynamixelWalker::reportMotorVelocities(uint16_t *motor_velocity) {
  int number_of_motors = 15;
  for (int i = 0; i < number_of_motors; i++) {
    // must convert 0-1023 to radian per second
    // after converting to RPM, convert to radians per second
    motor_velocity[i] = _motor_velocities[i];
  }
}

void MyDynamixelWalker::zeroFootPosition() {
  #if defined DEBUG
  Serial.println("zeroFootPosition");
  #endif
  _step = 0;
  calculateTrotAngles();
  for (int i = 0; i < 12; i++) {
    _motor_velocities[i] = 100;
  }
  calculateNewPositions();
  syncMotorVelocity(12,_dxl_motor_ids,_motor_velocities);
  // calculateNewPositions();
  updateRobotMotorPositions();
  // syncMotorRadianAngles(12, _dxl_motor_ids, _next_angle_values);
  // syncMotorPosition(12, _dxl_motor_ids, _last_motor_positions);
}

void MyDynamixelWalker::zeroGunAiming() {
  uint8_t dxl_ids [2] = { _dxl_motor_ids[12], _dxl_motor_ids[13] };
  gunAimLR = 512;
  gunAimUD = 512;
  uint16_t pan_and_tilt[2] = {512, 512};
  syncMotorPosition(2, dxl_ids, pan_and_tilt);
}

void MyDynamixelWalker::moveRobotViaWalk(float seconds_per_step){
  #if defined DEBUG
  Serial.println("------------------");
  Serial.println("-moveRobotViaWalk-");
  Serial.println("------------------");
  Serial.print("-Step: ");
  Serial.print(_step);
  Serial.println();
  Serial.print("-currentMoving: ");
  Serial.print(currentMoving);
  Serial.println();
  Serial.print("Foot_Zeroed_status:");
  Serial.print(_foot_Zeroed[0]);
  Serial.print(_foot_Zeroed[1]);
  Serial.print(_foot_Zeroed[2]);
  Serial.print(_foot_Zeroed[3]);
  Serial.println();
  Serial.println("------------------");
  #endif

  if (_step%2 == 0) {
    _lastMFB = currentMFB;
    _lastMLR = currentMLR;
    _lastTLR = currentTLR;
  }

    if (_lastMFB == 0 and _lastMLR == 0 and _lastTLR == 0) {
      if (_foot_Zeroed[0] == 1 and _foot_Zeroed[1] == 1 and _foot_Zeroed[2] == 1 and _foot_Zeroed[3] == 1) {
        _step = 0;
        currentMoving = 0;
      }
      else {
        _step = _step % 8 + 1;
        currentMoving = 0;
      }
    }
    else {
      currentMoving = 1;
      _step = _step % 8 + 1;
    }
    #if defined DEBUG
    Serial.println("");
    Serial.println("*calculateWalkAngles");
    #endif
    calculateWalkAngles();
    #if defined DEBUG
    Serial.println("*calculateNewPositions");
    #endif
    calculateNewPositions();
    #if defined DEBUG
    Serial.println("*calculateNewVelocities");
    #endif
    // calculateNewVelocities(0.125);
    calculateNewVelocities(seconds_per_step);
    #if defined DEBUG
    Serial.println("*updateRobotMotorVelocities");
    #endif
    updateRobotMotorVelocities();
    #if defined DEBUG
    Serial.println("*updateRobotMotorPositions");
    #endif
    updateRobotMotorPositions();
}

void MyDynamixelWalker::moveRobotViaTrot(float seconds_per_step){
  if (_step == 0 or _step == 3 or _step == 7) {
    _lastMFB = currentMFB;
    _lastMLR = currentMLR;
    _lastTLR = currentTLR;
    if (_lastMFB == 0 and _lastMLR == 0 and _lastTLR == 0) {
      currentMoving = 0;
      _step = 0;
    }
    else {
      _step = _step % 8 + 1;
      currentMoving = 1;
    }

  }
  else {
    _step = _step % 8 + 1;
  }
  #if defined DEBUG
  Serial.println("");
  Serial.println("*calculateTrotAngles");
  #endif
  calculateTrotAngles();
  #if defined DEBUG
  Serial.println("*calculateNewPositions");
  #endif
  calculateNewPositions();
  #if defined DEBUG
  Serial.println("*calculateNewVelocities");
  // calculateNewVelocities(0.125);
  #endif
  calculateNewVelocities(seconds_per_step);
  #if defined DEBUG
  Serial.println("*updateRobotMotorVelocities");
  #endif
  updateRobotMotorVelocities();
  #if defined DEBUG
  Serial.println("*updateRobotMotorPositions");
  #endif
  updateRobotMotorPositions();
}

void MyDynamixelWalker::moveTrotToStep(uint8_t step,float time){
  _step = step;
  _lastMFB = currentMFB;
  _lastMLR = currentMLR;
  _lastTLR = currentTLR;

  calculateTrotAngles();
  calculateNewPositions();
  calculateNewVelocities(time/2);
  updateRobotMotorVelocities();
  updateRobotMotorPositions();
}

void MyDynamixelWalker::moveWalkToStep(uint8_t step,float time){
  _step = step;
  _lastMFB = currentMFB;
  _lastMLR = currentMLR;
  _lastTLR = currentTLR;

  calculateWalkAngles();
  calculateNewPositions();
  calculateNewVelocities(time/2);
  updateRobotMotorVelocities();
  updateRobotMotorPositions();
}


void MyDynamixelWalker::calculateNewVelocities(float time) {
  // and how do I calculate the joint velocities?
  float distance = 0;
  float speed = 0;
  #if defined DEBUG
  Serial.print("calculateNewVelocities");
  #endif
  for (int i = 0; i < 12; i++) {
    #if defined DEBUG
    if (i%3 == 0)  Serial.println("");
    Serial.print(i);
    Serial.print(") ");
    #endif
    distance = abs( float(_next_motor_positions[i] - _last_motor_positions[i]) * 300.0 / 1023.0) ; //convert from units to degrees
    #if defined DEBUG
    Serial.print("distance:");
    Serial.print(distance);
    #endif
    speed = distance / time * 60.0 / 360.0; //convert from degees per second to revolutions per minute;
    #if defined DEBUG
    Serial.print(" speed:");
    Serial.print(speed);
    Serial.print(" ");
    #endif
    _motor_velocities[i] = int(speed * 9); // convert from rpm to dynamixel unit speed (1 rpm = 9 units or 1 unit = .111 rpm)
    #if defined DEBUG
    Serial.print(" ");
    Serial.print(_motor_velocities[i]);
    #endif
  }
  // and how do I calculate the gun motor velocities?
  // _motor_velocities[12] = ;
  // _motor_velocities[13] = ;
  _motor_velocities[14] = loaderSpeed;
  // Serial.print("calculateNewVelocities");
  // for (int i = 0; i < 15; i++) {
  //   if (i%3 == 0) Serial.println("");
  //   Serial.print(" ");
  //   Serial.print(i);
  //   Serial.print(") ");
  //   Serial.print(_motor_velocities[i]);
  // }
  #if defined DEBUG
  Serial.println("");
  #endif
}

void MyDynamixelWalker::updateGunAim(){

}

void MyDynamixelWalker::updateRobotMotorVelocities() {
  _motor_velocities[14] = loaderSpeed;
  #if defined DEBUG
  Serial.print("-updateRobotMotorVelocities");
  for (int i = 0; i < 15; i++) {
    if (i%3 == 0) Serial.println("");
    Serial.print(" ");
    Serial.print(i);
    Serial.print(") ");
    Serial.print(_motor_velocities[i]);
  }
  #endif
  syncMotorVelocity(_number_of_motors,_dxl_motor_ids,_motor_velocities);
  #if defined DEBUG
  Serial.println("");
  #endif
}
void MyDynamixelWalker::calculateNewPositions() {
  #if defined DEBUG
  Serial.print("-calculateNewPositions");
  #endif
  for (int i = 0; i < 12; i++) {
    #if defined DEBUG
    if (i%3 == 0) Serial.println("");
    Serial.print(" ");
    Serial.print(i);
    Serial.print(")");
    #endif
    _last_motor_positions[i] = _next_motor_positions[i];
    _next_motor_positions[i] = int(_next_angle_values[i]/PI*180/300*1023)+512;
    #if defined DEBUG
    Serial.print(" old:");
    Serial.print(_last_motor_positions[i]);
    Serial.print(" new:");
    Serial.print(_next_motor_positions[i]);
    #endif
  }
  #if defined DEBUG
  Serial.println("");
  #endif
}

void MyDynamixelWalker::updateRobotMotorPositions() {
  _next_motor_positions[12] = gunAimLR;
  _next_motor_positions[13] = gunAimUD;
  #if defined DEBUG
  Serial.print("-updateRobotMotorPositions");
  for (int i = 0; i < 14; i++) {
    if (i%3 == 0) Serial.println("");
    Serial.print(" ");
    Serial.print(i);
    Serial.print(") ");
    Serial.print(_next_motor_positions[i]);
  }
  #endif
  syncMotorPosition(_number_of_motors-1,_dxl_motor_ids,_next_motor_positions);
  #if defined DEBUG
  Serial.println("");
  #endif
}


float MyDynamixelWalker::__angle_wrap(float angle) {
  // if (angle < 0) angle+=2*PI;
  float returnvalue = (fmod(angle + PI,2 * PI) - PI);
  if (returnvalue < -PI) returnvalue += 2*PI;
  return returnvalue;
  // return angle;
}

int MyDynamixelWalker::return_next_motor_positions(uint8_t motor_number) {
  return _next_motor_positions[motor_number];
}
int MyDynamixelWalker::return_last_motor_positions(uint8_t motor_number) {
  return _last_motor_positions[motor_number];
}
float MyDynamixelWalker::return_next_motor_angles(uint8_t motor_number) {
  return _next_angle_values[motor_number];
}
