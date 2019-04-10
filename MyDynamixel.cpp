#include "MyDynamixel.h"

MyDynamixel::MyDynamixel() {
}

MyDynamixel::~MyDynamixel() {
  uint8_t dxl_error = 0;                          // Dynamixel error
  _packetHandler->write1ByteTxRx(_portHandler, DXL_ID_BROADCAST, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  _portHandler->closePort();
}

//initalization functions
void MyDynamixel::initializeDynamixels() {
  uint8_t dxl_error = 0;                          // Dynamixel error
  _portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  _packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  _portHandler->openPort();
  _portHandler->setBaudRate(BAUDRATE);
  _packetHandler->write1ByteTxRx(_portHandler, DXL_ID_BROADCAST, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  _portHandler->openPort();
}
void MyDynamixel::initializeOpenCRsIMU() {
  // cIMU _imu;
  _imu.begin();
}

//single-motor functions
int MyDynamixel::pingMotor(uint8_t dxl_id, uint16_t &dxl_model_number, uint8_t &dxl_error) {
  int dxl_comm_result = COMM_TX_FAIL;
  dxl_comm_result = _packetHandler->ping(_portHandler, dxl_id, &dxl_model_number, &dxl_error);
  // if (dxl_comm_result != COMM_SUCCESS)
  // {
  //   Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
  // }
  // else if (dxl_error != 0)
  // {
  //   Serial.print(_packetHandler->getRxPacketError(dxl_error));
  // }
  return dxl_comm_result;
}
// int MyDynamixel::getMotorVoltage(uint8_t dxl_id, float &float_voltage, uint8_t &dxl_error) {
//   int dxl_comm_result = COMM_TX_FAIL;
//   uint8_t dxl_voltage = 0;
//   dxl_comm_result = _packetHandler->read1ByteTxRx(_portHandler, dxl_id, ADDR_PRESENT_VOLTAGE, &dxl_voltage, &dxl_error);
//   // if (dxl_comm_result != COMM_SUCCESS)
//   // {
//   //   // Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
//   // }
//   // else if (dxl_error != 0)
//   // {
//   //   // Serial.print(_packetHandler->getRxPacketError(dxl_error));
//   // }
//   float_voltage = (float) dxl_voltage/10.0;
//   return dxl_comm_result;
// }
int MyDynamixel::getMotorVoltage(uint8_t dxl_id, uint8_t &voltage, uint8_t &dxl_error) {
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_voltage = 0;
  dxl_comm_result = _packetHandler->read1ByteTxRx(_portHandler, dxl_id, ADDR_PRESENT_VOLTAGE, &dxl_voltage, &dxl_error);
  // if (dxl_comm_result != COMM_SUCCESS)
  // {
  //   // Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
  // }
  // else if (dxl_error != 0)
  // {
  //   // Serial.print(_packetHandler->getRxPacketError(dxl_error));
  // }
  voltage = dxl_voltage;
  return dxl_comm_result;
}
// int MyDynamixel::getMotorTemperature(uint8_t dxl_id, float &float_temperature, uint8_t &dxl_error) {
//   int dxl_comm_result = COMM_TX_FAIL;
//   uint8_t dxl_temperature = 0;
//   dxl_comm_result = _packetHandler->read1ByteTxRx(_portHandler, dxl_id, ADDR_PRESENT_TEMPERATURE, &dxl_temperature, &dxl_error);
//   // if (dxl_comm_result != COMM_SUCCESS)
//   // {
//   //   // Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
//   // }
//   // else if (dxl_error != 0)
//   // {
//   //   // Serial.print(_packetHandler->getRxPacketError(dxl_error));
//   // }
//   float_temperature = (float) dxl_temperature;
//   return dxl_comm_result;
// }
int MyDynamixel::getMotorTemperature(uint8_t dxl_id, uint8_t &temperature, uint8_t &dxl_error) {
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_temperature = 0;
  dxl_comm_result = _packetHandler->read1ByteTxRx(_portHandler, dxl_id, ADDR_PRESENT_TEMPERATURE, &dxl_temperature, &dxl_error);
  // if (dxl_comm_result != COMM_SUCCESS)
  // {
  //   // Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
  // }
  // else if (dxl_error != 0)
  // {
  //   // Serial.print(_packetHandler->getRxPacketError(dxl_error));
  // }
  temperature = dxl_temperature;
  return dxl_comm_result;
}
int MyDynamixel::setMotorPosition(uint8_t dxl_id, uint16_t motor_position, uint8_t &dxl_error) {
  int dxl_comm_result = COMM_TX_FAIL;
  dxl_comm_result = _packetHandler->write2ByteTxRx(_portHandler, dxl_id, ADDR_AX_GOAL_POSITION, motor_position, &dxl_error);
  // if (dxl_comm_result != COMM_SUCCESS)
  // {
  //   // Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
  // }
  // else if (dxl_error != 0)
  // {
  //   // Serial.print(_packetHandler->getRxPacketError(dxl_error));
  // }
  return dxl_comm_result;
}
int MyDynamixel::setMotorVelocity(uint8_t dxl_id, uint16_t motor_velocity, uint8_t &dxl_error) {
  int dxl_comm_result = COMM_TX_FAIL;
  dxl_comm_result = _packetHandler->write2ByteTxRx(_portHandler, dxl_id, ADDR_AX_GOAL_SPEED, motor_velocity, &dxl_error);
  // if (dxl_comm_result != COMM_SUCCESS)
  // {
  //   // Serial.print(_packetHandler->getTxRxResult(dxl_comm_result));
  // }
  // else if (dxl_error != 0)
  // {
  //   // Serial.print(_packetHandler->getRxPacketError(dxl_error));
  // }
  return dxl_comm_result;
}

//multi-motor functions
void MyDynamixel::syncMotorVelocity(int number_of_motors, uint8_t *dxl_motor_ids, uint16_t *motor_velocity) {
  dynamixel::GroupSyncWrite groupSyncWriteMotorVelocity(_portHandler, _packetHandler, ADDR_AX_GOAL_SPEED, LEN_AX_GOAL_SPEED);
  for (int i = 0; i < number_of_motors; i++) {
    groupSyncWriteMotorVelocity.addParam(dxl_motor_ids[i], Lo_Hi_Value(motor_velocity[i]));
  }
  groupSyncWriteMotorVelocity.txPacket();
  groupSyncWriteMotorVelocity.clearParam();
}
void MyDynamixel::syncMotorPosition(int number_of_motors, uint8_t *dxl_motor_ids, uint16_t *motor_position) {
  dynamixel::GroupSyncWrite groupSyncWriteMotorPosition(_portHandler, _packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
  for (int i = 0; i < number_of_motors; i++) {
    groupSyncWriteMotorPosition.addParam(dxl_motor_ids[i], Lo_Hi_Value(motor_position[i]));
  }
  groupSyncWriteMotorPosition.txPacket();
  groupSyncWriteMotorPosition.clearParam();
}
void MyDynamixel::syncMotorRadianAngles(int number_of_motors, uint8_t *dxl_motor_ids, float *motor_angles) {
  dynamixel::GroupSyncWrite groupSyncWriteMotorPosition(_portHandler, _packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
  uint16_t *motor_position;
  for (int i = 0; i < number_of_motors; i++) {
    motor_position[i] = int(motor_angles[i]/PI*180/300*1023)+512;
    groupSyncWriteMotorPosition.addParam(dxl_motor_ids[i], Lo_Hi_Value(motor_position[i]));
  }
  groupSyncWriteMotorPosition.txPacket();
  groupSyncWriteMotorPosition.clearParam();
}
void MyDynamixel::syncMotorDegreeAngles(int number_of_motors, uint8_t *dxl_motor_ids, int *motor_angles) {
  dynamixel::GroupSyncWrite groupSyncWriteMotorPosition(_portHandler, _packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
  uint16_t *motor_position;
  for (int i = 0; i < number_of_motors; i++) {
    motor_position[i] = int(motor_angles[i]/300.0*1023)+512;
    groupSyncWriteMotorPosition.addParam(dxl_motor_ids[i], Lo_Hi_Value(motor_position[i]));
  }
  groupSyncWriteMotorPosition.txPacket();
  groupSyncWriteMotorPosition.clearParam();
}
// void MyDynamixel::reportMotorTemperatures(int number_of_motors, uint8_t *dxl_motor_ids, float *temperature) {
//     int dxl_comm_result = COMM_TX_FAIL;
//     uint8_t dxl_error = 0;
//     for (int i=0; i<number_of_motors; i++)
//     {
//       getMotorTemperature(dxl_motor_ids[i],temperature[i], dxl_error);
//     }
//     // return 0; //not checking for failure in the many reads
// }
// void MyDynamixel::reportMotorVoltages(int number_of_motors, uint8_t *dxl_motor_ids, float *voltage) {
//   int dxl_comm_result = COMM_TX_FAIL;
//   uint8_t dxl_error = 0;
//   for (int i=0; i<number_of_motors; i++)
//   {
//     getMotorVoltage(dxl_motor_ids[i],voltage[i], dxl_error);
//   }
//   // return 0; //not checking for failure in the many reads
// }
void MyDynamixel::reportMotorTemperatures(int number_of_motors, uint8_t *dxl_motor_ids, uint8_t *temperature) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    for (int i=0; i<number_of_motors; i++)
    {
      getMotorTemperature(dxl_motor_ids[i],temperature[i], dxl_error);
    }
    // return 0; //not checking for failure in the many reads
}
void MyDynamixel::reportMotorVoltages(int number_of_motors, uint8_t *dxl_motor_ids, uint8_t *voltage) {
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  for (int i=0; i<number_of_motors; i++)
  {
    getMotorVoltage(dxl_motor_ids[i],voltage[i], dxl_error);
  }
  // return 0; //not checking for failure in the many reads
}

//math functions
void MyDynamixel::calcMotorSpeed( int posOld, int posNew, int &speed, float time) {
  if (time == 0){
    return;
  } else
  {
    speed = (int) abs( posNew - posOld )  / time;
  }
}
void MyDynamixel::calcMotorSpeed(int number_of_motors, int *posOld, int *posNew, int *speed, float time) {
  for (int i = 0; i < number_of_motors; i++) {
    speed[i] = (int) abs( posNew[i] - posOld[i] )  / time;
  }
}


//board functions
float MyDynamixel::returnOpenCRVoltage(void) {
  int adc_value;
  float vol_value;

  adc_value = analogRead(BDPIN_BAT_PWR_ADC);
  vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value / 100.0; //returns voltage in Volts

  return vol_value;
}
// int MyDynamixel::getUsbConnected(void) {
//   return (int) vcp_is_connected();
// }

//support functions for internal use
uint8_t * MyDynamixel::Lo_Hi_Value(int value) {
  static uint8_t return_value[2];
  return_value[0] = DXL_LOBYTE(value);
  return_value[1] = DXL_HIBYTE(value);
  return return_value;
}
