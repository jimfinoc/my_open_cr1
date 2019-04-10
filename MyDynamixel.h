#ifndef MyDynamixel_h
#define MyDynamixel_h

#include <DynamixelSDK.h>
#include <Robot_Functions.h>
#include <IMU.h>



class MyDynamixel {
  private:
    dynamixel::PortHandler *_portHandler;
    dynamixel::PacketHandler *_packetHandler;
    cIMU _imu;

    uint8_t * Lo_Hi_Value(int value) ;

  public:
    MyDynamixel();
    ~MyDynamixel();
    // used to initalize the motors. moved this out of the constructor to make it happen during the setup routine
    void initializeDynamixels();
    void initializeOpenCRsIMU();
    //this produces the model number of the dynamixel motor
    int pingMotor(uint8_t dxl_id, uint16_t &dxl_model_number, uint8_t &dxl_error);

    //this interacts with one motor and returns the individual desired value
    // int getMotorVoltage(uint8_t dxl_id, float &float_voltage, uint8_t &dxl_error);// void Sync03Spd(int spd11, int spd12, int spd13);
    // int getMotorTemperature(uint8_t dxl_id, float &float_temperature, uint8_t &dxl_error);// void Sync03Spd(int spd11, int spd12, int spd13);
    // int getMotorTemperature(uint8_t dxl_id, uint8_t &float_temperature, uint8_t &dxl_error);// void Sync03Spd(int spd11, int spd12, int spd13);
    int getMotorVoltage(uint8_t dxl_id, uint8_t &voltage, uint8_t &dxl_error);// void Sync03Spd(int spd11, int spd12, int spd13);
    int getMotorTemperature(uint8_t dxl_id, uint8_t &temperature, uint8_t &dxl_error);// void Sync03Spd(int spd11, int spd12, int spd13);

    //this interacts with one motor and sets the individual desired value
    int setMotorPosition(uint8_t dxl_id, uint16_t motor_position, uint8_t &dxl_error);
    int setMotorVelocity(uint8_t dxl_id, uint16_t motor_velocity, uint8_t &dxl_error);

    //this interacts with a group of motors and at the same time, updates their values
    void syncMotorVelocity(int number_of_motors, uint8_t *dxl_motor_ids, uint16_t *motor_velocity);
    void syncMotorPosition(int number_of_motors, uint8_t *dxl_motor_ids, uint16_t *motor_position);
    void syncMotorDegreeAngles(int number_of_motors, uint8_t *dxl_motor_ids, int *motor_angles);
    void syncMotorRadianAngles(int number_of_motors, uint8_t *dxl_motor_ids, float *motor_angles);

    //this polls the motors individually to get the desired values
    // void reportMotorTemperatures(int number_of_motors, uint8_t *dxl_motor_ids, float *temperature);
    // void reportMotorVoltages(int number_of_motors, uint8_t *dxl_motor_ids, float *voltage);
    void reportMotorTemperatures(int number_of_motors, uint8_t *dxl_motor_ids, uint8_t *temperature);
    void reportMotorVoltages(int number_of_motors, uint8_t *dxl_motor_ids, uint8_t *voltage);

    // these calculate speeds
    void calcMotorSpeed( int posOld, int posNew, int &speed, float time);
    void calcMotorSpeed(int number_of_motors, int *posOld, int *posNew, int *speed, float time);
    // void calcMotorSpeed( int posNew, int &speed, float time);
    // void calcMotorSpeed(int number_of_motors, int *posNew, int *speed, float time);


    // these routines are for the individal opencr board
    float returnOpenCRVoltage(void);

    // void Sync14Pos(int pos11, int pos12, int pos13, int pos21, int pos22, int pos23,int pos31,int pos32,int pos33, int pos41,int pos42,int pos43,int pos71,int pos72);
    // void reportMotorTemperaturesAndVoltages(int number_of_motors, uint8_t dxl_motor_ids[], char* name[], float temperature[], float voltage[]);
    // void Sync03Pos(int pos11, int pos12, int pos13);
    // void Sync03Spd(int *spd);
    // void Sync04Spd(int spd81, int spd82, int spd83, int spd84);
    // void Sync04Spd(int spd[4]);
    // void Sync03Pos(int *pos);
    // void Sync12Spd(int spd11, int spd12, int spd13, int spd21, int spd22, int spd23,int spd31,int spd32,int spd33, int spd41,int spd42,int spd43);
    // void Sync12Spd(int *spd);
    // void Sync14Spd(int spd11, int spd12, int spd13, int spd21, int spd22, int spd23,int spd31,int spd32,int spd33, int spd41,int spd42,int spd43,int spd71,int spd72);
    // void Sync15Spd(int spd11, int spd12, int spd13, int spd21, int spd22, int spd23,int spd31,int spd32,int spd33, int spd41,int spd42,int spd43,int spd71,int spd72, int spd73);
    // void Sync12Pos(int pos11, int pos12, int pos13, int pos21, int pos22, int pos23,int pos31,int pos32,int pos33, int pos41,int pos42,int pos43);
    // void Sync12Pos(int *pos);


};

#endif
