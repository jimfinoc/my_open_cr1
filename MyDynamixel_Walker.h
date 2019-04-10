#ifndef My_Dynamixel_Walker_h
#define My_Dynamixel_Walker_h

#include "MyDynamixel.h"
#include <math.h>

#define FR 0
#define FL 1
#define BR 2
#define BL 3

#define DEBUG 0

class MyDynamixelWalker : public MyDynamixel {
  private:
    uint8_t _number_of_motors = 15;
    uint8_t _dxl_motor_ids[15]; //includes the loader
    uint16_t _motor_velocities[15]; //includes the loader
    uint16_t _last_motor_positions[14];
    float _lastMFB = 0;
    float _lastMLR = 0;
    float _lastTLR = 0;
    uint16_t _next_motor_positions[14];
    float _next_angle_values[14];
    uint8_t _step = 0;

    uint8_t _stopping = 0;
    uint8_t _foot_Zeroed[4] = {0,0,0,0};
    float _foot_position_x[4] = {0,0,0,0};
    float _foot_position_y[4] = {0,0,0,0};
    float _foot_position_z[4] = {0,0,0,0};

    float _link_a = 0.078; //in meters
    float _link_b = 0.13; //in meters
    // float _verticle_displacement = 0.05; //in meters
    // float _up_leg = 0.03;
    float _up_leg = 0.06;
    float _step_rotation = 45.0; //degrees
    float _step_x_distance = 0.06; //in meters
    float _step_y_distance = 0.06; //in meters
    float _joint_placement_x[4] = { 0.1,  0.1, -0.1, -0.1};
    float _joint_placement_y[4] = {-0.1,  0.1, -0.1,  0.1};
    float _joint_placement_z = 0.0415;
    float _initial_foot_placement_x[4] = { 0.15,  0.15, -0.15, -0.15}; //in meters
    float _initial_foot_placement_y[4] = {-0.15,  0.15, -0.15,  0.15}; //in meters
    float _initial_foot_placement_z = -0.1; //in meters
    float _leg_rotation[4] = {PI*7/4,PI*1/4,PI*5/4,PI*3/4}; // in radians FR,FL,BR,BL
    float _twist_degrees[4] = {0,0,0,0};
    float _moving_x[4][9] = { {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0}};
    float _moving_y[4][9] = { {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0}};
    float _moving_z[4][9] = { {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0},
                              {0, 0,0,0,0, 0,0,0,0}};
    float _turning_x[4][9] = { {0, 0,0,0,0, 0,0,0,0},
                               {0, 0,0,0,0, 0,0,0,0},
                               {0, 0,0,0,0, 0,0,0,0},
                               {0, 0,0,0,0, 0,0,0,0}};
    float _turning_y[4][9] = { {0, 0,0,0,0, 0,0,0,0},
                               {0, 0,0,0,0, 0,0,0,0},
                               {0, 0,0,0,0, 0,0,0,0},
                               {0, 0,0,0,0, 0,0,0,0}};
    float __angle_wrap(float angle);


  public:
    float currentMFB = 0;
    float currentMLR = 0;
    float currentTLR = 0;
    uint8_t currentMoving = 0;
    // float aimLR = 0;
    // float aimUD = 0;
    uint16_t loaderSpeed = 0;
    uint16_t gunAimLR = 512;
    uint16_t gunAimUD = 512;
    MyDynamixelWalker();

    // void initializeDynamixelWalker(int number_of_motors, uint8_t *dxl_motor_ids);
    void initializeRobot(uint8_t number_of_motors, uint8_t *dxl_motor_ids);
    void configureTrotFeetPositions();
    void configureWalkFeetPositions();
    void calculateTrotAngles();
    void calculateWalkAngles();
    void updateRobotMotorPositions();
    void updateRobotMotorVelocities();
    void calculateNewVelocities(float time);
    void calculateNewPositions();
    // void moveMotorPosition(uint16_t *motor_position, float time);
    void setLoaderSpeed(uint16_t loader_speed);
    void reportLastMotorPosition(uint16_t *motor_position);

    void zeroFootPosition();
    void zeroGunAiming();
    void moveRobotViaTrot(float seconds_per_step);
    void moveRobotViaWalk(float seconds_per_step);
    void moveTrotToStep(uint8_t step,float time);
    void moveWalkToStep(uint8_t step,float time);
    void updateGunAim();

    int return_last_motor_positions(uint8_t motor_number);
    int return_next_motor_positions(uint8_t motor_number);
    float return_next_motor_angles(uint8_t motor_number);

    void reportNextMotorPositionFloats(float *motor_position);
    void reportMotorVelocitiesFloats(float *motor_velocity);
    void reportNextMotorPosition(uint16_t *motor_position);
    void reportMotorVelocities(uint16_t *motor_velocity);

};

#endif
