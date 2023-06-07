#!/usr/bin/python3 

# Nano Example 1: Speed Control

# Date:    Jun. 6th, 2023
# Author:  Jerry Peng
 
from lib.NanoBot import NanoBot
import time


# assign serial port to robot. 
# ATTENTION: Change the parameter "/dev/cu.SLAB_USBtoUART" below 
#            according to your own computer OS system. Open whatever port is 7Bot on your computer.
# Usually:  "/dev/cu.SLAB_USBtoUART" on Mac OS
#           "/dev/ttyUSB0" on Linux
#           'COM1' on Windows
bot = NanoBot("/dev/cu.SLAB_USBtoUART") 

# 1. read robot infos
print("Device Code: ", bot.getDeviceCode())
print("Version: ", bot.getVersion())
print("MAC: ", bot.getMAC())
print("ID: ", bot.getID())
print("Version: ", bot.getVersion())

# 2. speed control
bot.setVelocityL(50)
bot.setVelocityR(50)
time.sleep(2)
bot.setVelocity(0)
time.sleep(1)
bot.setVelocity(-50)
time.sleep(1)
bot.setVelocity(0)
time.sleep(1)

