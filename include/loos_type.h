#ifndef LOOS_TYPE_H_
#define LOOS_TYPE_H_

#include <Arduino.h>

#define  SLAVE_ADDRESS      0x48

#define MAXBYTEEXPECTED 4

// I2C Register Addresses
#define UID_ADDR 0x00               //R 2
#define STATUS_ADDR 0x01            //R-W 1

#define MOTORCTRL_ADDR 0x02         //R-W 1

#define JAM_THRESH_CPS_ADDR 0x03    //R-W 2 //below this count, the jam state would be raised and system will be put into service mode
#define COUNT_MM_ADDR 0x04          //R-W 2 
#define MAX_FLUSH_COUNT_ADDR 0x05   //R-W 2
#define MAX_FILM_COUNT_ADDR 0x06    //R-W 4

#define FILM_LEFT_ADDR 0x07         //R-W 4 // current film count
#define MOTOR_STATE_ADDR 0x08       //R 2
#define TEMP_ADDR 0x09              //R 2

#define LT_FLUSH_ADDR 0x10          //R 4
#define LT_FILM_USED_ADDR 0x11      //R 4
#define LT_BLOCK_ADDR  0x12         //R 2
#define LT_SER_ADDR 0x13            //R 2

#define ACTUAL_FILM_ADDR 0x14       //R-W-2
#define PPR_MM_ADDR 0x15            //R-W-2
#define JAM_RPM_ADDR 0x16           //R-W-2

// EEPROM ADDRESSES
#define EE_IS_INIT_ADDR 0x00
#define EE_UID_0_ADDR 0x01
#define EE_UID_1_ADDR 0x02
#define EE_STATUS_ADDR 0x03

#define EE_FILM_LEFT_0_ADDR 0x04
#define EE_FILM_LEFT_1_ADDR 0x05
#define EE_FILM_LEFT_2_ADDR 0x06
#define EE_FILM_LEFT_3_ADDR 0x07

#define EE_MAX_FILM_0_ADDR 0x08
#define EE_MAX_FILM_1_ADDR 0x09
#define EE_MAX_FILM_2_ADDR 0x0A
#define EE_MAX_FILM_3_ADDR 0x0B

#define EE_LT_FLUSHES_0_ADDR 0x10
#define EE_LT_FLUSHES_1_ADDR 0x11
#define EE_LT_FLUSHES_2_ADDR 0x12
#define EE_LT_FLUSHES_3_ADDR 0x13

#define EE_LT_FILM_USED_0_ADDR 0x14
#define EE_LT_FILM_USED_1_ADDR 0x15
#define EE_LT_FILM_USED_2_ADDR 0x16
#define EE_LT_FILM_USED_3_ADDR 0x17

#define EE_LT_BLOCK_0_ADDR 0x18
#define EE_LT_BLOCK_1_ADDR 0x19

#define EE_LT_SER_0_ADDR 0x1A
#define EE_LT_SER_1_ADDR 0x1B

#define EE_ACTUAL_FILM_0_ADDR 0x1C
#define EE_ACTUAL_FILM_1_ADDR 0x1D

#define EE_PPR_MM_0_ADDR 0x1E
#define EE_PPR_MM_1_ADDR 0x1F

#define EE_JAM_RPM_0_ADDR 0x20
#define EE_JAM_RPM_1_ADDR 0x21


union T_TwoBytesData
{
  struct
  {
    byte byte_0: 8;
    byte byte_1: 8;
  } s_twobytes;
  unsigned short u_twobytes;
};

union T_FourBytesData
{
  struct
  {
    byte byte_0: 8;
    byte byte_1: 8;
    byte byte_2: 8;
    byte byte_3: 8;
  } s_fourbytes;
  unsigned long u_fourbytes;
};

union T_FourBytesData_Signed
{
  struct
  {
    byte byte_0: 8;
    byte byte_1: 8;
    byte byte_2: 8;
    byte byte_3: 8;
  } s_fourbytes;
  signed long u_fourbytes;
};

// internal tracking data

union T_StatusByte
{
  struct
  {
    byte state : 2;
    byte IsInit: 1;
    byte busy : 1;
    byte jam : 1;
    byte film : 1;
    byte fan : 1;
    byte light : 1;
  }s_status;
  byte u_status;
};

enum e_StateType{S_Run, S_Service, S_Stop, S_UN};
const char* e_StateTypeToString[] = {"Run", "Service", "Reserved", "Unknown"};

enum e_FilmStateType{NotEMPTY, EMPTY};
const char* e_FilmStateTypeToString[] = {"NotEMPTY", "EMPTY"};

enum e_EngagedFlagType{Engaged, Vacant};
const char* e_EngagedFlagTypeToString[] = {"Engaged", "Vacant"};

enum e_ErrorFlagType{OK, NOK};
const char* e_ErrorTypeToString[] = {"OK", "NOK"};

enum e_PeriphStateType{ON, OFF};
const char* e_PeriphStateTypeToString[] = {"ON", "OFF"};

union T_MotorControlByte
{
  struct
  {
    byte m_state : 2;
    byte flush   : 1;
  }s_motorcontrol;
  byte u_motorcontrol;
};

#endif /* LOOS_TYPE_H_ */