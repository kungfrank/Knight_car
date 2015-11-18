#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

import time
import atexit
import numpy
import warnings

class DCMotorInterface:
    __deadZone = 35       # Minimum speed to win static friction  
    __maxMotorSpeed = 255 # Maximum speed that can be commanded

    def __init__(self):
        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.rearMotor = self.mh.getMotor(1)
        self.frontMotor = self.mh.getMotor(2)
        self.rearMotorMode = Adafruit_MotorHAT.RELEASE

    # Speed has to be in [-1, 1], sign determines bwd/fwd

    def setSpeed(self, speed):
        # Warning the user
        if abs(speed)>1:
            warnings.warn("Input speed has to be in [-1, 1]. Clamping it.")

        # Clamping speed value to [-1, 1]
        speed = numpy.clip(speed, -1, 1)

        if abs(speed) < 1e-4:
            self.rearMotorMode = Adafruit_MotorHAT.RELEASE
        elif speed > 0:
            self.rearMotorMode = Adafruit_MotorHAT.FORWARD
        elif speed < 0: 
            self.rearMotorMode = Adafruit_MotorHAT.BACKWARD


        self.rearMotor.setSpeed(int(round(\
            abs(speed)*(DCMotorInterface.__maxMotorSpeed-DCMotorInterface.__deadZone)\
            +DCMotorInterface.__deadZone)));
        
        self.rearMotor.run(self.rearMotorMode);
        
    # def setSteerAngle(angle): 
    # TODO this has to be implemented (maybe using a separate control thread)
    # once the potentiometer data is available

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        self.rearMotor.run(Adafruit_MotorHAT.RELEASE)
        self.frontMotor.run(Adafruit_MotorHAT.RELEASE)
        
    def __del__(self):
        self.turnOffMotors()