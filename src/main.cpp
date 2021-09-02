#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "loos_type.h"
#include "vnh7100motor.h"
#include "pins.h"
#include "flushfilm.h"
#include <TaskScheduler.h>

#define INIT_EEPROM
#define MAX_FILM 216000  //192000 -> 12m// 13.7m -> 212,800
#define COUNT_CUTOFF 1920 //keep same
#define SET_UID 0x9999  //replace by 0x0001
#define MAX_LOW_RPM_COUNT_JAMMED_FLUSH 10
//ISRs
signed long encoderCount = 0;

void requestEvent();
void receiveEvent(int bytesReceived);
void encoderEvent();

void update_ui_percent();
void motor_control();
void flush_stop();
void forced_motor_stop();

long last_checked_millis;

#define SLAVE_ADDRESS 0x48

// i2c Registers
T_TwoBytesData UID_REG;

T_StatusByte STATUS_REG;
T_MotorControlByte MOTORCTRL_REG;

T_TwoBytesData JAM_THRESH_CPS_REG;
T_TwoBytesData COUNT_MM_REG;
T_TwoBytesData MAX_FLUSH_COUNT_REG;
T_FourBytesData MAX_FILM_COUNT_REG;

T_FourBytesData_Signed FILM_LEFT_REG;
T_TwoBytesData MOTOR_STATE_REG;
T_TwoBytesData TEMP_REG;

// EEPROM REGISTERS
byte EEIsInit;
T_TwoBytesData EEUID;
T_FourBytesData_Signed EEFilmLeft;
T_FourBytesData EEMaxFilm;
T_StatusByte EEStatus;

// EEPROM LT REGISTERS
T_FourBytesData LT_FLUSHES;
T_FourBytesData LT_FILM_USED;
T_TwoBytesData LT_BLOCKAGES;
T_TwoBytesData LT_SERVICES;

// Globals
byte REG_ADDRESS;
bool ADDRESS_RECEIVED = false;
bool NEED_EEPROM_INIT = true;
signed long previous_film_left = 0;

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI oled(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);
VNH7100Motor flushmotor(MOTOR_INA, MOTOR_INB, MOTOR_PWM, MOTOR_SEL0, MOTOR_CS);
FlushFilm film(MAX_FILM);

// TASKs
Task percentage_update_task(200, TASK_FOREVER, &update_ui_percent);
Task motor_control_task(50, TASK_FOREVER, &motor_control);

Task flush_stop_task(4000, TASK_FOREVER, &flush_stop);
Task forced_motor_stop_task(10000, TASK_FOREVER, &forced_motor_stop);

int LowRPMCountFoward = 0;
int LowRPMCountBackward = 0;

const double FORWARD_JAM_RPM_CUTOFF = -26.0;//-30.0;  //this is backward  
const double BACKWARD_JAM_RPM_CUTOFF = 26.0;//33.0;  //this is forward  

Scheduler runner;

void setup()
{
  byte motorfault = 0x00;
  Serial.begin(9600);
  pinMode(STATUS_LED, OUTPUT);

  delay(2000);
  Serial.print("Fault: ");
  motorfault = flushmotor.init();
  Serial.println(motorfault, HEX);
  //Do a system Check

  // Startup oled
  oled.begin();
  oled.clearBuffer();
  // Set a suitable font. This font will be used for U8x8log
  oled.setDrawColor(1);
  oled.drawHLine(0, 14, 127);
  oled.drawHLine(0, 60, 127);
  oled.drawVLine(0, 14, 46);
  oled.drawVLine(127, 14, 46);
  oled.setFont(u8g2_font_6x12_tf);      // choose a suitable font
  oled.drawStr(0, 12, "SUID:");         // write something to the internal memory // ends at 30
  oled.setFont(u8g2_font_profont12_mf); // choose a suitable font
  oled.drawStr(24, 64, " Loowatt Ltd ");
  oled.setFont(u8g2_font_helvB18_tr);
  oled.drawUTF8(86, 48, "%");
  oled.sendBuffer();

  Wire.setClock(400000);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  Serial.println("--------- I2C -----------");

  EEPROM.read(EE_IS_INIT_ADDR);

  delay(100);

  if (EEPROM.read(EE_IS_INIT_ADDR) != 0xAA)
  {
    /* code */
    EEPROM.write(EE_IS_INIT_ADDR, 0xAA); // is init flat

    T_TwoBytesData W_UID;
    W_UID.u_twobytes = short(SET_UID);
    EEPROM.write(EE_UID_0_ADDR, W_UID.s_twobytes.byte_0);
    EEPROM.write(EE_UID_1_ADDR, W_UID.s_twobytes.byte_1);

    T_FourBytesData_Signed W_FILM;
    W_FILM.u_fourbytes = MAX_FILM; // in terms of encoder pulse count
    EEPROM.write(EE_FILM_LEFT_0_ADDR, W_FILM.s_fourbytes.byte_0);
    EEPROM.write(EE_FILM_LEFT_1_ADDR, W_FILM.s_fourbytes.byte_1);
    EEPROM.write(EE_FILM_LEFT_2_ADDR, W_FILM.s_fourbytes.byte_2);
    EEPROM.write(EE_FILM_LEFT_3_ADDR, W_FILM.s_fourbytes.byte_3);

    T_StatusByte W_STATUS;
    W_STATUS.s_status.state = S_Run;
    W_STATUS.s_status.IsInit = OK;
    W_STATUS.s_status.busy = Vacant;
    W_STATUS.s_status.jam = OK;
    W_STATUS.s_status.film = OK;
    W_STATUS.s_status.fan = OFF;
    W_STATUS.s_status.light = OFF;
    EEPROM.write(EE_STATUS_ADDR, W_STATUS.u_status);

    EEPROM.write(EE_MAX_FILM_0_ADDR, 0x00);
    EEPROM.write(EE_MAX_FILM_1_ADDR, 0xEE);
    EEPROM.write(EE_MAX_FILM_2_ADDR, 0x02);
    EEPROM.write(EE_MAX_FILM_3_ADDR, 0x00);

    EEPROM.write(EE_LT_FILM_USED_0_ADDR, 0);
    EEPROM.write(EE_LT_FILM_USED_1_ADDR, 0);
    EEPROM.write(EE_LT_FILM_USED_2_ADDR, 0);
    EEPROM.write(EE_LT_FILM_USED_3_ADDR, 0);

    EEPROM.write(EE_LT_FLUSHES_0_ADDR, 0);
    EEPROM.write(EE_LT_FLUSHES_1_ADDR, 0);
    EEPROM.write(EE_LT_FLUSHES_2_ADDR, 0);
    EEPROM.write(EE_LT_FLUSHES_3_ADDR, 0);

    EEPROM.write(EE_LT_BLOCK_0_ADDR, 0);
    EEPROM.write(EE_LT_BLOCK_1_ADDR, 0);

    EEPROM.write(EE_LT_SER_0_ADDR, 0);
    EEPROM.write(EE_LT_SER_1_ADDR, 0);
  }

  else
  {
    //Do Nothing
  }

  delay(100);

  EEIsInit = EEPROM.read(EE_IS_INIT_ADDR);

  EEUID.s_twobytes.byte_0 = EEPROM.read(EE_UID_0_ADDR);
  EEUID.s_twobytes.byte_1 = EEPROM.read(EE_UID_1_ADDR);

  EEFilmLeft.s_fourbytes.byte_0 = EEPROM.read(EE_FILM_LEFT_0_ADDR);
  EEFilmLeft.s_fourbytes.byte_1 = EEPROM.read(EE_FILM_LEFT_1_ADDR);
  EEFilmLeft.s_fourbytes.byte_2 = EEPROM.read(EE_FILM_LEFT_2_ADDR);
  EEFilmLeft.s_fourbytes.byte_3 = EEPROM.read(EE_FILM_LEFT_3_ADDR);

  EEMaxFilm.s_fourbytes.byte_0 = EEPROM.read(EE_MAX_FILM_0_ADDR);
  EEMaxFilm.s_fourbytes.byte_1 = EEPROM.read(EE_MAX_FILM_1_ADDR);
  EEMaxFilm.s_fourbytes.byte_2 = EEPROM.read(EE_MAX_FILM_2_ADDR);
  EEMaxFilm.s_fourbytes.byte_3 = EEPROM.read(EE_MAX_FILM_3_ADDR);

  EEStatus.u_status = EEPROM.read(EE_STATUS_ADDR);

  LT_FLUSHES.s_fourbytes.byte_0 = EEPROM.read(EE_LT_FLUSHES_0_ADDR);
  LT_FLUSHES.s_fourbytes.byte_1 = EEPROM.read(EE_LT_FLUSHES_1_ADDR);
  LT_FLUSHES.s_fourbytes.byte_2 = EEPROM.read(EE_LT_FLUSHES_2_ADDR);
  LT_FLUSHES.s_fourbytes.byte_3 = EEPROM.read(EE_LT_FLUSHES_3_ADDR);

  Serial.println(LT_FLUSHES.u_fourbytes, HEX);

  LT_BLOCKAGES.s_twobytes.byte_0 = EEPROM.read(EE_LT_BLOCK_0_ADDR);
  LT_BLOCKAGES.s_twobytes.byte_1 = EEPROM.read(EE_LT_BLOCK_1_ADDR);

  Serial.println(LT_BLOCKAGES.u_twobytes, HEX);

  LT_SERVICES.s_twobytes.byte_0 = EEPROM.read(EE_LT_SER_0_ADDR);
  LT_SERVICES.s_twobytes.byte_1 = EEPROM.read(EE_LT_SER_1_ADDR);

  Serial.println(LT_SERVICES.u_twobytes, HEX);

  if (EEIsInit == 0xAA)
  {
    STATUS_REG.u_status = EEStatus.u_status;

    UID_REG.u_twobytes = EEUID.u_twobytes;
    FILM_LEFT_REG.u_fourbytes = EEFilmLeft.u_fourbytes;
    Serial.println(FILM_LEFT_REG.u_fourbytes, DEC);
    Serial.println(UID_REG.u_twobytes, HEX);

    oled.setCursor(36, 12);
    oled.setFont(u8g2_font_6x12_tr);
    oled.print(UID_REG.u_twobytes, HEX);
    oled.sendBuffer();
  }
  else
  {
    STATUS_REG.u_status = 0xff;
    STATUS_REG.s_status.IsInit = e_ErrorFlagType(NOK);

    oled.setCursor(31, 12);
    oled.setFont(u8g2_font_6x12_tr);
    oled.print("Not Initialized");
    oled.sendBuffer();
  }

  runner.init();

  runner.addTask(percentage_update_task);
  runner.addTask(motor_control_task);
  runner.addTask(forced_motor_stop_task);
  runner.addTask(flush_stop_task);

  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB18_tr); // set the target font for the text width calculation

  film.init(FILM_LEFT_REG.u_fourbytes, 0x12345678);

  if (STATUS_REG.s_status.state == S_Run)
  {
    digitalWrite(STATUS_LED, HIGH);
  }
  else
  {
    digitalWrite(STATUS_LED, LOW);
  }

  // encoder Interrupt
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderEvent, RISING);
  percentage_update_task.enable();
  motor_control_task.enable();
}

void loop()
{
  runner.execute();
}

//################## Interrupt Routines ######################

void encoderEvent()
{
  if (flushmotor.MState == MFORWARD)
  {
    encoderCount = encoderCount - 1;
  }
  else if (flushmotor.MState == MBACKWARD)
  {
    encoderCount = encoderCount + 1;
  }
}

void requestEvent()
{
  // when requesting, address has to be received first
  switch (REG_ADDRESS)
  {
  case UID_ADDR:
    Wire.write(UID_REG.s_twobytes.byte_0);
    Wire.write(UID_REG.s_twobytes.byte_1);
    break;
  case STATUS_ADDR:
    Wire.write(STATUS_REG.u_status);
    // statements
    break;
  case MOTORCTRL_ADDR:
    Wire.write(MOTORCTRL_REG.u_motorcontrol);
    break;
  case JAM_THRESH_CPS_ADDR:
    Wire.write(JAM_THRESH_CPS_REG.s_twobytes.byte_0);
    Wire.write(JAM_THRESH_CPS_REG.s_twobytes.byte_1);
    break;
  case COUNT_MM_ADDR:
    Wire.write(COUNT_MM_REG.s_twobytes.byte_0);
    Wire.write(COUNT_MM_REG.s_twobytes.byte_1);
    break;
  case MAX_FLUSH_COUNT_ADDR:
    Wire.write(MAX_FLUSH_COUNT_REG.s_twobytes.byte_0);
    Wire.write(MAX_FLUSH_COUNT_REG.s_twobytes.byte_1);
    break;
  case MAX_FILM_COUNT_ADDR:
    Wire.write(EEMaxFilm.s_fourbytes.byte_0);
    Wire.write(EEMaxFilm.s_fourbytes.byte_1);
    Wire.write(EEMaxFilm.s_fourbytes.byte_2);
    Wire.write(EEMaxFilm.s_fourbytes.byte_3);
    break;
  case FILM_LEFT_ADDR:
    Wire.write(FILM_LEFT_REG.s_fourbytes.byte_0);
    Wire.write(FILM_LEFT_REG.s_fourbytes.byte_1);
    Wire.write(FILM_LEFT_REG.s_fourbytes.byte_2);
    Wire.write(FILM_LEFT_REG.s_fourbytes.byte_3);
    break;
  case MOTOR_STATE_ADDR:
    Wire.write(MOTOR_STATE_REG.s_twobytes.byte_0);
    Wire.write(MOTOR_STATE_REG.s_twobytes.byte_1);
    break;
  case TEMP_ADDR:
    Wire.write(TEMP_REG.s_twobytes.byte_0);
    Wire.write(TEMP_REG.s_twobytes.byte_1);
    break;
  case LT_FLUSH_ADDR:
    LT_FLUSHES.s_fourbytes.byte_0 = EEPROM.read(EE_LT_FLUSHES_0_ADDR);
    LT_FLUSHES.s_fourbytes.byte_1 = EEPROM.read(EE_LT_FLUSHES_1_ADDR);
    LT_FLUSHES.s_fourbytes.byte_2 = EEPROM.read(EE_LT_FLUSHES_2_ADDR);
    LT_FLUSHES.s_fourbytes.byte_3 = EEPROM.read(EE_LT_FLUSHES_3_ADDR);

    Wire.write(LT_FLUSHES.s_fourbytes.byte_0);
    Wire.write(LT_FLUSHES.s_fourbytes.byte_1);
    Wire.write(LT_FLUSHES.s_fourbytes.byte_2);
    Wire.write(LT_FLUSHES.s_fourbytes.byte_3);
    break;
  case LT_FILM_USED_ADDR:
    LT_FILM_USED.s_fourbytes.byte_0 = EEPROM.read(EE_LT_FILM_USED_0_ADDR);
    LT_FILM_USED.s_fourbytes.byte_1 = EEPROM.read(EE_LT_FILM_USED_1_ADDR);
    LT_FILM_USED.s_fourbytes.byte_2 = EEPROM.read(EE_LT_FILM_USED_2_ADDR);
    LT_FILM_USED.s_fourbytes.byte_3 = EEPROM.read(EE_LT_FILM_USED_3_ADDR);

    Wire.write(LT_FILM_USED.s_fourbytes.byte_0);
    Wire.write(LT_FILM_USED.s_fourbytes.byte_1);
    Wire.write(LT_FILM_USED.s_fourbytes.byte_2);
    Wire.write(LT_FILM_USED.s_fourbytes.byte_3);
    break;
  case LT_BLOCK_ADDR:
    LT_BLOCKAGES.s_twobytes.byte_0 = EEPROM.read(EE_LT_BLOCK_0_ADDR);
    LT_BLOCKAGES.s_twobytes.byte_1 = EEPROM.read(EE_LT_BLOCK_1_ADDR);

    Wire.write(LT_BLOCKAGES.s_twobytes.byte_0);
    Wire.write(LT_BLOCKAGES.s_twobytes.byte_1);

    break;
  case LT_SER_ADDR:
    LT_SERVICES.s_twobytes.byte_0 = EEPROM.read(EE_LT_SER_0_ADDR);
    LT_SERVICES.s_twobytes.byte_1 = EEPROM.read(EE_LT_SER_1_ADDR);

    Wire.write(LT_SERVICES.s_twobytes.byte_0);
    Wire.write(LT_SERVICES.s_twobytes.byte_1);
    break;
  default:
    // statements
    break;
  }
  ADDRESS_RECEIVED = false;
}

void dump(int bytesReceived)
{
  byte dump;

  for (int i = 0; i < bytesReceived; i++)
  {
    dump = Wire.read();
  }
}

void receiveEvent(int bytesReceived)
{

  if (ADDRESS_RECEIVED)
  {
    switch (REG_ADDRESS)
    {
    case UID_ADDR:
      dump(bytesReceived); //cannot be written
      break;
    case STATUS_ADDR:
      if (bytesReceived == sizeof(STATUS_REG.u_status))
      {
        T_StatusByte status_only;
        status_only.u_status = Wire.read();
        STATUS_REG.s_status.state = status_only.s_status.state; // only state is writable flags are read-only

        if (STATUS_REG.s_status.state == S_Run)
        {
          STATUS_REG.s_status.jam = OK;
          digitalWrite(STATUS_LED, HIGH);
        }
        else
        {
          digitalWrite(STATUS_LED, LOW);
        }

        EEPROM.write(EE_STATUS_ADDR, STATUS_REG.u_status);
      }
      else
      {
        dump(bytesReceived);
      }

      break;
    case MOTORCTRL_ADDR:
      if (bytesReceived == sizeof(MOTORCTRL_REG.u_motorcontrol))
      {
        MOTORCTRL_REG.u_motorcontrol = Wire.read();
      }
      else
      {
        dump(bytesReceived);
      }
      break;

    case JAM_THRESH_CPS_ADDR:
      if (bytesReceived == sizeof(JAM_THRESH_CPS_REG.u_twobytes))
      {
        JAM_THRESH_CPS_REG.s_twobytes.byte_0 = Wire.read();
        JAM_THRESH_CPS_REG.s_twobytes.byte_1 = Wire.read();
      }
      else
      {
        dump(bytesReceived);
      }
      break;

    case COUNT_MM_ADDR:
      if (bytesReceived == sizeof(COUNT_MM_REG.u_twobytes))
      {
        COUNT_MM_REG.s_twobytes.byte_0 = Wire.read();
        COUNT_MM_REG.s_twobytes.byte_1 = Wire.read();
      }
      else
      {
        dump(bytesReceived);
      }
      break;

    case MAX_FLUSH_COUNT_ADDR:
      if (bytesReceived == sizeof(MAX_FLUSH_COUNT_REG.u_twobytes))
      {
        MAX_FLUSH_COUNT_REG.s_twobytes.byte_0 = Wire.read();
        MAX_FLUSH_COUNT_REG.s_twobytes.byte_1 = Wire.read();
      }
      else
      {
        dump(bytesReceived);
      }
      break;

    case FILM_LEFT_ADDR:
      Serial.println(bytesReceived);
      if (bytesReceived == sizeof(FILM_LEFT_REG.u_fourbytes))
      {
        FILM_LEFT_REG.s_fourbytes.byte_0 = Wire.read();
        FILM_LEFT_REG.s_fourbytes.byte_1 = Wire.read();
        FILM_LEFT_REG.s_fourbytes.byte_2 = Wire.read();
        FILM_LEFT_REG.s_fourbytes.byte_3 = Wire.read();

        film.init(FILM_LEFT_REG.u_fourbytes, millis());
        LT_SERVICES.u_twobytes = LT_SERVICES.u_twobytes + 1;

        STATUS_REG.s_status.film = NotEMPTY;

        EEPROM.write(EE_FILM_LEFT_0_ADDR, FILM_LEFT_REG.s_fourbytes.byte_0);
        EEPROM.write(EE_FILM_LEFT_1_ADDR, FILM_LEFT_REG.s_fourbytes.byte_1);
        EEPROM.write(EE_FILM_LEFT_2_ADDR, FILM_LEFT_REG.s_fourbytes.byte_2);
        EEPROM.write(EE_FILM_LEFT_3_ADDR, FILM_LEFT_REG.s_fourbytes.byte_3);

        EEPROM.write(EE_LT_SER_0_ADDR, LT_SERVICES.s_twobytes.byte_0);
        EEPROM.write(EE_LT_SER_1_ADDR, LT_SERVICES.s_twobytes.byte_1);

        EEPROM.write(EE_STATUS_ADDR, STATUS_REG.u_status);
      }
      else
      {
        dump(bytesReceived);
      }

      break;

    default:
      dump(bytesReceived);
      break;
    }

    ADDRESS_RECEIVED = false;
  }
  else
  {
    REG_ADDRESS = Wire.read();
    ADDRESS_RECEIVED = true;
  }
}

//################## Scheduler Tasks ############################

void update_ui_percent()
{
  bool OnChange = (previous_film_left != FILM_LEFT_REG.u_fourbytes);
  previous_film_left = FILM_LEFT_REG.u_fourbytes;
  if (OnChange)
  {
    float mm_film_count = FILM_LEFT_REG.u_fourbytes / 13.5; // arduino can't handle big division it seems
    float percentage = (mm_film_count / 16000.0) * 100.0;

    int percentage_int = int(percentage);

    oled.clearBuffer();
    oled.setCursor(36, 48);
  

    if (percentage_int < 100)
    {
      oled.print(' ');
      oled.print(' ');
    }
    if (percentage_int < 10)
    {
      oled.print(' ');
      oled.print(' ');
    }

    if (percentage_int >= 100)
    {
      percentage_int = 100;
    }

    oled.print(percentage_int);

    oled.updateDisplayArea(1, 3, 9, 4); //x,
  }
}

void motor_control()
{
  bool MStateChanged = MOTORCTRL_REG.s_motorcontrol.m_state != flushmotor.MState; //indicate first state change

  switch (MOTORCTRL_REG.s_motorcontrol.m_state)
  {

  case MSTOP:

    if (MStateChanged)
    {
      flushmotor.stop();
      flushmotor.RPM = 0.0;

      if (flush_stop_task.isEnabled())
      {
        flush_stop_task.disable();

        LT_FLUSHES.u_fourbytes = LT_FLUSHES.u_fourbytes + 1;
        EEPROM.write(EE_LT_FLUSHES_0_ADDR, LT_FLUSHES.s_fourbytes.byte_0);
        EEPROM.write(EE_LT_FLUSHES_1_ADDR, LT_FLUSHES.s_fourbytes.byte_1);
        EEPROM.write(EE_LT_FLUSHES_2_ADDR, LT_FLUSHES.s_fourbytes.byte_2);
        EEPROM.write(EE_LT_FLUSHES_3_ADDR, LT_FLUSHES.s_fourbytes.byte_3);
      }

      if (forced_motor_stop_task.isEnabled())
      {
        forced_motor_stop_task.disable();
      }

      EEPROM.write(EE_FILM_LEFT_0_ADDR, FILM_LEFT_REG.s_fourbytes.byte_0);
      EEPROM.write(EE_FILM_LEFT_1_ADDR, FILM_LEFT_REG.s_fourbytes.byte_1);
      EEPROM.write(EE_FILM_LEFT_2_ADDR, FILM_LEFT_REG.s_fourbytes.byte_2);
      EEPROM.write(EE_FILM_LEFT_3_ADDR, FILM_LEFT_REG.s_fourbytes.byte_3);
    }
    break;

  case MBACKWARD:

    if (MStateChanged)
    {
      flushmotor.backward();

      if (MOTORCTRL_REG.s_motorcontrol.flush)
      {
        flush_stop_task.enableDelayed();
      }
      else
      {
        forced_motor_stop_task.enableDelayed();
      }
    }

    // increase motor speed slowly
    break;

  case MFORWARD:

    if (MStateChanged)
    {
      flushmotor.forward();

      if (MOTORCTRL_REG.s_motorcontrol.flush)
      {
        flush_stop_task.enableDelayed();
      }
      else
      {
        forced_motor_stop_task.enableDelayed();
      }
    }
    // increase motor speed slowly
    break;
  case MBRAKE:
    // nyi
    break;
  }

  if (MOTORCTRL_REG.s_motorcontrol.m_state == MFORWARD || MOTORCTRL_REG.s_motorcontrol.m_state == MBACKWARD)
  {
    if (flushmotor.pwm_value < 240)
    {
      flushmotor.pwm_value = flushmotor.pwm_value + 10;
      flushmotor.setMSpeed();
    }
    signed long count = encoderCount;
    encoderCount = 0;
    film.updateFilmLeft(count);
    if (count > 0)
    {
      LT_FILM_USED.u_fourbytes = LT_FILM_USED.u_fourbytes + count;

      EEPROM.write(EE_LT_FILM_USED_0_ADDR, LT_FILM_USED.s_fourbytes.byte_0);
      EEPROM.write(EE_LT_FILM_USED_1_ADDR, LT_FILM_USED.s_fourbytes.byte_1);
      EEPROM.write(EE_LT_FILM_USED_2_ADDR, LT_FILM_USED.s_fourbytes.byte_2);
      EEPROM.write(EE_LT_FILM_USED_3_ADDR, LT_FILM_USED.s_fourbytes.byte_3);
    }
    FILM_LEFT_REG.u_fourbytes = film.getFilmLeft();

    if (FILM_LEFT_REG.u_fourbytes < COUNT_CUTOFF)
    {
      STATUS_REG.s_status.film = EMPTY;
      Serial.println(e_FilmStateTypeToString[STATUS_REG.s_status.film]);
    }

    long time_elapsed = millis() - last_checked_millis;
    double temporalmultiplier = 60.0 / (time_elapsed / 1000.0);
    last_checked_millis = millis();
    double rpm = ((count / 16.0) * temporalmultiplier) / 149.25;
    flushmotor.RPM = rpm;

    Serial.println(flushmotor.RPM, DEC);

    if (MOTORCTRL_REG.s_motorcontrol.m_state == MFORWARD)
    {
      if (flushmotor.pwm_value > 240 && flushmotor.RPM > FORWARD_JAM_RPM_CUTOFF) // value needs to be calibrated with current draw most probably
      {
        LowRPMCountFoward = LowRPMCountFoward + 1;
        Serial.print("FCount : ");
        Serial.println(LowRPMCountFoward);
      }
      else
      {
        if (MOTORCTRL_REG.s_motorcontrol.m_state == MSTOP)
        {
          LowRPMCountFoward = 0;
        }
        
      }
    }

    if (MOTORCTRL_REG.s_motorcontrol.m_state == MBACKWARD)
    {
      if (flushmotor.pwm_value > 240 && flushmotor.RPM < BACKWARD_JAM_RPM_CUTOFF) // value needs to be calibrated with current draw most probably
      {
        LowRPMCountBackward = LowRPMCountBackward + 1;
        Serial.print("BCount :");
        Serial.println(LowRPMCountBackward);
      }
      else
      {
        if (MOTORCTRL_REG.s_motorcontrol.m_state == MSTOP)
        {
          LowRPMCountFoward = 0;
        }
      }
    }

    //int currentdraw = flushmotor.getCurrent();

    if (LowRPMCountFoward > MAX_LOW_RPM_COUNT_JAMMED_FLUSH || LowRPMCountBackward > MAX_LOW_RPM_COUNT_JAMMED_FLUSH)
    {
      STATUS_REG.s_status.jam = NOK;
      MOTORCTRL_REG.s_motorcontrol.m_state = MSTOP;
      LT_BLOCKAGES.u_twobytes = LT_BLOCKAGES.u_twobytes + 1;

      EEPROM.write(EE_LT_BLOCK_0_ADDR, LT_BLOCKAGES.s_twobytes.byte_0);
      EEPROM.write(EE_LT_BLOCK_1_ADDR, LT_BLOCKAGES.s_twobytes.byte_1);

      LowRPMCountFoward = 0;
      LowRPMCountBackward = 0;
    }
    // need to check both current and RPM to check whether jamming has occurred
  }
}

void flush_stop()
{
  if (flushmotor.MState != MSTOP)
  {
    MOTORCTRL_REG.s_motorcontrol.m_state = MSTOP;
  }
}

void forced_motor_stop()
{
  if (flushmotor.MState != MSTOP)
  {
    MOTORCTRL_REG.s_motorcontrol.m_state = MSTOP;
  }
}