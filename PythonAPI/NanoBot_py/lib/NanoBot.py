#!/usr/bin/python3

# Project: Python Library for NanoBot
# Version: 1.0
# Author: Artyom Liu & Jerry Peng
# Date:   Jun. 6th, 2023

# This project is under coding
# 2021-3-24: add CRC

import serial # Install on Linux: sudo apt-get install python3-serial


BAUD_RATE = 115200      # baud rate of robot serial port

# Register ID
# ROM
DEVICE_TYPE_ID = 0
VERSION_ID = 1
MAC_ID = 2
# EEPROM
EEPROM_LEN = 2
EEPROM_ID = 11
DEVICE_ID = 11
BAUDRATE_ID = 12
# RAM
EEPROM_LOCK_ID = 28
VELOCITY_L_ID = 30
VELOCITY_R_ID = 31
BTN_ENABLE_ID = 66


class NanoBot:

    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD_RATE, timeout=0.2)

    #######################################################################
    # Get Functions #

    # get decice code

    def getDeviceCode(self):
        return self.readReg(DEVICE_TYPE_ID, 1)[0]

    # get version
    def getVersion(self):
        return self.readReg(VERSION_ID, 1)[0]/10

    # get MAC address(lenmgth: 6 bytes)
    def getMAC(self):
        return self.listToString(self.readReg(MAC_ID, 6))
    # Function to convert

    def listToString(self, l: list):
        # initialize an empty string
        str1 = ""
        # traverse in the string
        for ele in l:
            str1 += hex(ele)[2:]
        # return string
        return str1

    # get ID
    def getID(self):
        return self.readReg(DEVICE_ID, 1)[0]


    #######################################################################
    # Set Functions #

    # set decice code

    def setID(self, ID: int):
        self.writeReg(DEVICE_ID, [ID])

    # set EEPROM write lock: 0-lock off(enable writting), 1-lock on(protect EEPROM)
    def setLock(self, lock: int):
        self.writeReg(EEPROM_LOCK_ID, [lock])

    # [function] Set linear velocity
    # [input]    velocity, range: [-168, 168]（Unit: mm/s)

    def setVelocityL(self, v: int):
        data = v+128
        self.writeReg(VELOCITY_L_ID, [data])

    # [function] Set linear velocity
    # [input]    velocity, range: [-168, 168]（Unit: mm/s)
    def setVelocityR(self, v: int):
        data = v+128
        self.writeReg(VELOCITY_R_ID, [data])

      # velocity test for goAhead & goBack motion

    def setVelocity(self, v: int):
        data = []
        data.append(v+128)
        data.append(v+128)
        self.writeReg(VELOCITY_L_ID, data)



    # EEPROM data init, this function will erase offset data.
    def EEPROMinit(self):
        data=[0, 0]
        self.setLock(0)
        self.writeReg(EEPROM_ID, data)
        self.setLock(1)



    #######################################################################
    # Serial Comunication #

    def readReg(self, addr, num):
        # asking for register information
        buf=[0x03, addr & 0xff, num & 0xff]
        self.writeSerial(buf)

        ret_pack=self.readSerial()
        if (ret_pack.pop(0) != 0x03):
            raise serial.SerialException("mismatching pack type")
        return ret_pack[2:]

    def writeReg(self, addr: int, data: list):
        buf=[0x04, addr & 0xff, len(data) & 0xff]
        for d in data:
            buf.append(d & 0xff)
        self.writeSerial(buf)
        # information feedback from robot is not required so far




    def readSerial(self):
        cnt=0
        # read pack head
        while (True):
            tmp=self.ser.read()
            # print(tmp)
            if (tmp == b'\xaa'):
                tmp=self.ser.read()
                if (tmp == b'\x77'):
                    break
            cnt += 1
            if (50 == cnt):
                raise serial.SerialTimeoutException()

        # from here data is to be returned
        tmp=self.ser.read(3)
        ret=[0xaa, 0x77, tmp[0], tmp[1], tmp[2]]
        tmp=self.ser.read(ret[4] + 2)
        for d in tmp:
            ret.append(d)
        crc=self.CRC16_MODBUS(ret[0:-2])
        if ((crc & 0xff == ret[-2]) and ((crc >> 8) & 0xff == ret[-1])):
            return ret[2:-2]
        else:
            # print(ret)
            raise serial.SerialException("data corrupted")

    def writeSerial(self, data: list):
        buf=[0xaa, 0x77]
        for tmp in data:
            buf.append(tmp)

        crc=self.CRC16_MODBUS(buf)
        buf.append(crc & 0xff)
        buf.append((crc >> 8) & 0xff)

        return self.ser.write(bytes(buf)) == len(buf)



    #######################################################################
    # CRC #
    def invert8(self, val):
        ret=i=0
        while (i < 8):
            ret |= ((val >> i) & 0x01) << (7 - i)
            i += 1
        return ret & 0xff

    def invert16(self, val):
        ret=i=0
        while (i < 16):
            ret |= ((val >> i) & 0x01) << (15 - i)
            i += 1
        return ret & 0xffff

    def CRC16_MODBUS(self, data: list):
        wCRCin=0xffff
        wCPoly=0x8005

        for tmp in data:
            tmp=self.invert8(tmp)
            wCRCin ^= (tmp << 8) & 0xffff
            i=0
            while (i < 8):
                i += 1
                if ((wCRCin & 0x8000) != 0):
                    wCRCin=0xffff & ((wCRCin << 1) & 0xffff) ^ wCPoly
                else:
                    wCRCin=(wCRCin << 1) & 0xffff
        wCRCin=self.invert16(wCRCin)
        return wCRCin
