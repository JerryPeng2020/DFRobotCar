/****************************************************
 * Copyright (C) 2016-2023 Pinecone.AI - All Rights Reserved
 * 
 * Project: DFRobot Arduino Class
 * Version: 1.0
 * 
 * Date:   Jun. 6th, 2023
 * Author: Jerry Peng
 * Web:    www.pinecone.ai
 *  
/***************************************************/




/******************************
nanoBot class
Date: 2023.6.6


  ESP 32 data length:

              bool	1
           boolean	1
              byte	1
              char	1
     unsigned char	1
           uint8_t	1
             short	2
          uint16_t	2
              word	4
               int	4
      unsigned int	4
            size_t	4
             float	4
              long	4
     unsigned long	4
          uint32_t	4
            double	8
         long long	8
unsigned long long	8
          uint64_t	8
  /***************************************************/


#ifndef _nanoBot_H
#define _nanoBot_H



#include "EEPROM.h"


/* Pre-defined Parameters */
#define CONVEYOR_MOTOR_ID 0
#define CTRLLOOP_PERIOD 33   // Unit:ms


/* Hardware Define */
#define STBY 21

#define MOTOR_AN1 23
#define MOTOR_AN2 22

#define MOTOR_BN1 32
#define MOTOR_BN2 33

/* Command Registers */
#define DEVICE_TYPE 22  // DFRobot 
#define VERSION 10     // version 1.0

// EEPROM： 17 for register(currently, 2 used)
#define EEPROM_SIZE 17
#define REG_EEPROM_SIZE 2

// Command Header
#define BEGIN_FLAG_0 0xAA
#define BEGIN_FLAG_1 0x77

/* Register ID */
// ROM
#define DEVICE_TYPE_ID 0
#define VERSION_ID 1
#define MAC_ID 2
// EEPROM
#define EEPROM_ID 11
#define DEVICE_ID 11
#define BAUDRATE_ID 12
// RAM 
#define EEPROM_LOCK_ID 28
//
#define VELOCITY_L_ID 30
#define VELOCITY_R_ID 31
#define BTN_ENABLE_ID 66

///////////////////////////////////////////////////////////////////////////////
/*              LEDC PWM           */
///////////////////////////////////////////////////////////////////////////////
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_8_BIT 8 // use 8 bit precission for LEDC timer
#define LEDC_BASE_FREQ 500 // use 500 Hz as a LEDC base frequency


class nanoBot{

  private:

  public:

    /* Hardware parameters */
    // wheel
    void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax);
    void setMotorSpeed(int ID, int speed);
    
   

    /* Device Functions */
    nanoBot();
    void init();
    void firmwareSystem();
    void receiveCommand();

 
    // ctrl loop
    long long loopTimeCnt;      // ctrlLoop 执行的周期计数器
    void ctrlLoop();
    void feedback();


    /* Command Parameters */
    uint64_t chipID;
    // UART
    bool beginFlag = false;
    bool haveInstruction = false;
    int instruction = 0;
    int cnt = 0;
    int dataBuf[120];
    int rxBuf_pre;
    // CRC
    void sendData(uint8_t *data, int len);
    uint16_t CRC16_MODBUS(uint8_t *data, int len);
    uint8_t InvertUint8( uint8_t dBuf);
    uint16_t InvertUint16( uint16_t dBuf);
    // command register
    uint8_t comReg[100];
    

};


#endif
