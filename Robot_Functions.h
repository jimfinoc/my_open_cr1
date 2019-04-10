#ifndef Robot_Functions_h
#define Robot_Functions_h


// Control table address
#define ADDR_MODEL_NUMBER               0
#define AX12_MODEL_NUMBER               12
#define AX18_MODEL_NUMBER               18
#define MX12_MODEL_NUMBER               360

#define ADDR_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION           30
#define ADDR_GOAL_SPEED              32
#define ADDR_PRESENT_POSITION        36
#define ADDR_PRESENT_VELOCITY        38
#define ADDR_PRESENT_VOLTAGE         42
#define ADDR_PRESENT_TEMPERATURE     43
#define ADDR_MX_GOAL_ACCELERATION       73

#define ADDR_AX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_PRESENT_POSITION        36
#define ADDR_AX_GOAL_SPEED              32
#define ADDR_AX_PRESENT_SPEED           38

// Data Byte Length
#define LEN_AX_GOAL_POSITION            2
#define LEN_AX_PRESENT_POSITION         2
#define LEN_AX_GOAL_SPEED               2
#define LEN_AX_PRESENT_SPEED            2


//Pan and Tilt Joint Limits
#define AX_STEPS            1024
#define AX_DEGREES          300
#define MX_STEPS            4096
#define MX_DEGREES          360
//Movement modifiers

#define DEGREES_TO_MOVE_VERTICLE           10
#define DEGREES_TO_MOVE_HORIZONTAL         25
#define PI                       3.1415926536

// Acceleration goal for the MX motor.
#define MX_ACCELERATION_GOAL        1
#define TORQUE_ENABLE               1
#define TORQUE_DISABLE              1


//Quick Aiming
#define CENTER_PAN_DEGREE_POSITION 0
#define CENTER_TILT_DEGREE_POSITION 0
#define LEFT_PAN_DEGREE_POSITION 90
#define LEFT_TILT_DEGREE_POSITION 0
#define RIGHT_PAN_DEGREE_POSITION (-LEFT_PAN_DEGREE_POSITION)
#define RIGHT_TILT_DEGREE_POSITION 0
// Aiming Limits
#define LEFT_PAN_DEGREE_LIMIT 148
#define UP_TILT_DEGREE_LIMIT 30
#define RIGHT_PAN_DEGREE_LIMIT (-LEFT_PAN_DEGREE_LIMIT)
#define DOWN_TILT_DEGREE_LIMIT (-20)

#define TURN_ANGLE          15
#define STEP_DISTANCE       .010
// speed
#define FAST    200
#define SLOW    50

#define JOINT1_UP           0
#define JOINT2_UP           (-90 * PI / 180)
#define JOINT3_UP           (90 * PI / 180)

#define JOINT1_CENTERED     0
#define JOINT2_CENTERED     (-105 * PI / 180)
#define JOINT3_CENTERED     (75 * PI / 180)

#define TIME                (.250 * 60) // Conversion from seconds to minutes
#define UNIT_STEP_SPEED     (.111) // This is in RadiansPerSecond

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "OpenCR_DXL_Port"   // This definition only has a symbolic meaning and does not affect to any functionality
#define DEVICE_NAME ""

#define MOTOR_NUM 15
//Front Right Leg Motors
#define DXL_ID_11  11
#define DXL_ID_12  12
#define DXL_ID_13  13
//Front Left Leg Motors
#define DXL_ID_21  21
#define DXL_ID_22  22
#define DXL_ID_23  23
//BackLeft Leg Motors
#define DXL_ID_31  31
#define DXL_ID_32  32
#define DXL_ID_33  33
//Back Right Leg Motors
#define DXL_ID_41  41
#define DXL_ID_42  42
#define DXL_ID_43  43
//Pan Joint Motor
#define DXL_ID_71   71
#define DXL_ID_PAN  71
//Tilt Joint Motor
#define DXL_ID_72   72
#define DXL_ID_TILT 72
//Ammo Feeder Wheel Motor
#define DXL_ID_73   73
#define DXL_ID_LOAD 73

#define DXL_ID_81   81
#define DXL_ID_82   82
#define DXL_ID_83   83
#define DXL_ID_84   84

#define DXL_ID_FR   81
#define DXL_ID_FL   82
#define DXL_ID_BR   83
#define DXL_ID_BL   84

#define DXL_ID_BROADCAST   254


struct RobotJointsType {
  float joint11;
  float joint12;
  float joint13;
  float joint21;
  float joint22;
  float joint23;
  float joint31;
  float joint32;
  float joint33;
  float joint41;
  float joint42;
  float joint43;
} ;

struct RobotPositionType {
  float x1;
  float y1;
  float z1;
  float x2;
  float y2;
  float z2;
  float x3;
  float y3;
  float z3;
  float x4;
  float y4;
  float z4;
} ;

struct RobotCommandType {
  float twist_left_right;
  float move_forward_back;
  float move_right_left;
  bool sending_command;
  int step;
} ;


#endif
