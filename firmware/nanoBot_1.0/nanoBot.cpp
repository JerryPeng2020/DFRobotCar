// NanoBot Class
// Date: 2020.12.16

#include "nanoBot.h"

/* Constructor */
nanoBot::nanoBot() { }


void nanoBot::init() {

  /* Read ROM data to comReg(command register) */
  comReg[DEVICE_TYPE_ID] = DEVICE_TYPE;
  comReg[VERSION_ID] = VERSION;

  // Get chip ID (MAC address), length = 6 bytes
  chipID = ESP.getEfuseMac();
  for (int i = 0; i < 6; i++)
    comReg[MAC_ID + i] = chipID >> (40 - i * 8);

  /* EEPROM init and Read EEPROM data to comReg */
  //
  // indivudual EEPROM: will set every uninitialized byte(0x00) to 0xFF.
  //                    If they these bytes have been set to a value(include 0x00),
  //                    the stored data will not change anymore.
  if (!EEPROM.begin(EEPROM_SIZE))
  { /*  Serial.println("failed to initialise EEPROM"); */
  }
  else
  { /* Serial.println("Successfully initialise EEPROM"); */
  }

  // read EEPROM data to comReg
  for (int i = 0; i < REG_EEPROM_SIZE; i++)
  {
    comReg[EEPROM_ID + i] = EEPROM.read(i);
  }

  // Device Serial Port Init: read EEPROM(1) & setup serial port
  if (comReg[BAUDRATE_ID] == 0)
    Serial.begin(115200);
  else if (comReg[BAUDRATE_ID] == 3)
    Serial.begin(1000000);
  //  else if(buadrateIndex == 7) Serial.begin(9600);
  else
    Serial.begin(115200);

  /* Init comReg RAM data */
  comReg[EEPROM_LOCK_ID] = 1;  // eepromLock, 0-disable EEPROM write protect lock(can write),  1-enable it
  comReg[VELOCITY_L_ID] = 128; // velocity = comReg[VELOCITY_ID] - 128
  comReg[VELOCITY_R_ID] = 128; // velocity = comReg[VELOCITY_ID] - 128

  /* motor */
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  // Setup timer and attach timer to a led pin
  // 1:    channel
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(MOTOR_AN1, 0);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(MOTOR_AN2, 1);
  // 2
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(MOTOR_BN1, 2);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_8_BIT);
  ledcAttachPin(MOTOR_BN2, 3);
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Motion monitor  */

// motion control loop
void nanoBot::ctrlLoop() {
  // Serial 
  receiveCommand();

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Serial Port */


// scan and receive command from USB-UART port
void nanoBot::receiveCommand()
{
  /*  recive commands  */
  
  while (Serial.available() > 0)
  {
    // read data
    int rxBuf = Serial.read();
    if (!beginFlag)
    {
      beginFlag = (rxBuf_pre == BEGIN_FLAG_0 && rxBuf == BEGIN_FLAG_1) ? true : false; // Beginning Flag
    }
    else
    {
      if (instruction == 0)
        instruction = rxBuf;
      else
      {
        switch (instruction)
        {

        // Read register
        case 0x03:
          dataBuf[cnt++] = rxBuf;
          if (cnt >= 4)
          {
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            uint8_t data_read[] = {0xAA, 0x77, 0x03, dataBuf[0], dataBuf[1]};
            uint16_t crc = CRC16_MODBUS(data_read, 5);
            int high = crc / 256;
            int low = crc % 256;
            int addr = dataBuf[0]; // begin address
            int num = dataBuf[1];  // reg number

            // Serial.write(num);   // num debug

            // CRC check pass, send reg data
            if (low == dataBuf[2] && high == dataBuf[3])
            {
              // prepare data
              uint8_t data_send[5 + num];
              data_send[0] = 0xAA;
              data_send[1] = 0x77;
              data_send[2] = 0x03;
              data_send[3] = addr;
              data_send[4] = num;
              // update relevant registers
              for (int i = 0; i < num; i++)
              {  
                
                // if((addr+i) == MOTION_STATUS_FEEDBACK_ID)
                //   comReg[MOTION_STATUS_FEEDBACK_ID] = wheelL.motionStatus || wheelR.motionStatus;
                // else if((addr+i) == VELOCITY_L_FB_ID)
                //   comReg[VELOCITY_L_FB_ID] = wheelL.readVelocity()+128;  // !!! 存在离散化误差
                // else if((addr+i) == VELOCITY_R_FB_ID)
                //   comReg[VELOCITY_R_FB_ID] = wheelR.readVelocity()+128;  // !!! 存在离散化误差
                // else if((addr+i) == DISTANCE_L_FB_ID+1) {  
                //   int dist = round(wheelL.shiftDist);
                //   comReg[DISTANCE_L_FB_ID+1]  = abs(dist)/256%128;
                //   comReg[DISTANCE_L_FB_ID]    = abs(dist)%256;
                //   if(dist<0) comReg[DISTANCE_L_FB_ID+1] += 128;
                // }
                // else if((addr+i) == DISTANCE_R_FB_ID+1) {  
                //   int dist = round(wheelR.shiftDist);
                //   comReg[DISTANCE_R_FB_ID+1]  = abs(dist)/256%128;
                //   comReg[DISTANCE_R_FB_ID]    = abs(dist)%256;
                //   if(dist<0) comReg[DISTANCE_R_FB_ID+1] += 128;
                // }
              }
              // write data to send
              for (int i = 0; i < num; i++)
                data_send[5 + i] = comReg[addr + i];
              
              // send data with CRC inside
              sendData(data_send, num + 5);
            }
          }
          break;

        // write reg
        case 0x04:
          dataBuf[cnt++] = rxBuf;
          // get data length
          if (cnt >= 2)
          {
            if (cnt >= dataBuf[1] + 4)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int addr = dataBuf[0];
              int num = dataBuf[1];
              // data read
              uint8_t data_read[5 + num];
              data_read[0] = 0xAA;
              data_read[1] = 0x77;
              data_read[2] = 0x04;
              data_read[3] = addr;
              data_read[4] = num;

              for (int i = 0; i < num; i++)
              {
                data_read[5 + i] = dataBuf[2 + i];
              }
              // CRC check
              uint16_t crc = CRC16_MODBUS(data_read, num + 5);
              int high = crc / 256;
              int low = crc % 256;

              // CRC check pass, write reg
              if (low == dataBuf[num + 2] && high == dataBuf[num + 3])
              {
                // write comReg
                for (int i = addr; i < addr + num; i++)
                {
                  // update writeable addr data
                  if (i >= EEPROM_ID && i <= BTN_ENABLE_ID)
                    comReg[i] = dataBuf[2 + i - addr];

                  // immediate execute command
                  if (i == VELOCITY_L_ID) {
                    int v = comReg[i] - 128;
                    v= v*2;
                    setMotorSpeed(2, v);
                  }
                  else if (i == VELOCITY_R_ID) {
                    int v = comReg[i] - 128;
                     v= v*2;
                    setMotorSpeed(1, v);
                  }

                }

                // if need write EEPROM, check lock frist and then write EEPROM
                if (comReg[EEPROM_LOCK_ID] == 0)
                {
                  for (int i = EEPROM_ID; i < EEPROM_ID + REG_EEPROM_SIZE; i++)
                    if (i >= addr && i < addr + num)
                      EEPROM.write(i - EEPROM_ID, comReg[i]);
                }
                EEPROM.commit();
              }
            }
          }
          break;

        default:
          beginFlag = false;
          instruction = 0;
          cnt = 0;
          break;
        }
      }
    }

    rxBuf_pre = rxBuf;
  }

}


void nanoBot::feedback() 
{
      // prepare data
      /* int dataLength = 4;
      uint8_t data_send[5 + dataLength];
      data_send[0] = 0xAA;
      data_send[1] = 0x77;
      data_send[2] = 0x05;
      data_send[3] = MOTION_STATUS_FEEDBACK_ID;
      data_send[4] = dataLength;

      //
      comReg[MOTION_STATUS_FEEDBACK_ID] = motionStatus;
      comReg[VELOCITY_FEEDBACK_ID]      = readVelocity()+128;
      int dist = round(shiftDist);
      comReg[DISTANCE_FEEDBACK_ID]      = abs(dist)%256;
      comReg[DISTANCE_FEEDBACK_ID + 1]  = abs(dist)/256%128;
      if(dist<0) comReg[DISTANCE_FEEDBACK_ID + 1] = comReg[DISTANCE_FEEDBACK_ID + 1]+128;

      // 
      for (int i = 0; i < dataLength; i++)
      {
        data_send[5 + i] = comReg[MOTION_STATUS_FEEDBACK_ID + i];
      }
      sendData(data_send, 5 + dataLength); */
}
      




/////////////////////////////////////////////////////////////////////////////////////////////////////////
/* CRC & Data Send */

// Be care!!!! sizeof(data) do not work properly, maybe because of
// the uint8_t data is not treat correctly in sizeof().
void nanoBot::sendData(uint8_t *data, int len)
{
  uint16_t crc = CRC16_MODBUS(data, len);
  int high = crc / 256;
  int low = crc % 256;
  for (int i = 0; i < len; i++)
    Serial.write(data[i]);
  Serial.write(low);
  Serial.write(high);
}

uint16_t nanoBot::CRC16_MODBUS(uint8_t *data, int len)
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wbyte = 0;
  int j = 0;
  while (len > 0)
  {
    len--;
    wbyte = data[j++];
    wbyte = InvertUint8(wbyte);
    wCRCin ^= (wbyte << 8);
    for (int i = 0; i < 8; i++)
    {
      if ((wCRCin & 0x8000) != 0)
        wCRCin = uint16_t((wCRCin << 1) ^ wCPoly);
      else
        wCRCin = uint16_t(wCRCin << 1);
    }
  }
  wCRCin = InvertUint16(wCRCin);
  return (wCRCin);
}

uint8_t nanoBot::InvertUint8(uint8_t dBuf)
{
  int i;
  uint8_t tmp = 0;
  for (i = 0; i < 8; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (7 - i);
  }
  return tmp;
}

uint16_t nanoBot::InvertUint16(uint16_t dBuf)
{
  int i;
  uint16_t tmp;
  tmp = 0;
  for (i = 0; i < 16; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (15 - i);
  }
  return tmp;
}


////////////////////////////////////////////
/* Hardware */

// Arduino like analogWrite value has to be between 0 and valueMax
void nanoBot::ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255)
{
  // calculate duty, 255 from 2 ^ 8 - 1
  uint32_t duty = (255 / valueMax) * min(value, valueMax);
  // write duty to LEDC
  ledcWrite(channel, duty);
}

// ID [1, 2], speed [-255, 255]
void nanoBot::setMotorSpeed(int ID, int speed)
{
  int s = abs(speed);
  if (speed > 0)
  {
    ledcAnalogWrite((ID - 1) * 2, s);
    ledcAnalogWrite((ID - 1) * 2 + 1, 0);
  }
  else
  {
    ledcAnalogWrite((ID - 1) * 2, 0);
    ledcAnalogWrite((ID - 1) * 2 + 1, s);
  }
}