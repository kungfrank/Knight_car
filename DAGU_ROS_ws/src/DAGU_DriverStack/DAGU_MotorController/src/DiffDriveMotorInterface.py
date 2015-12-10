#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#

# ~~~~~ IMPORTANT !!! ~~~~~
#
# Make sure that the front motor is connected in such a way that a positive
# speed  causes an increase in the potentiometer reading!

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import atexit
import numpy
import warnings
import math

class DiffDriveMotorInterface:

    __leftDeadZone = 60        # Minimum speed for left motor  
    __leftMaxSpeed = 255       # Maximum speed for left motor

    __rightDeadZone = 60       # Minimum speed for right motor  
    __rightMaxSpeed = 255      # Maximum speed for right motor

    __ratioAxelRadius = 1.0    # Ratio between axel length and turning radius 

    def __init__(self, verbose=False, debug=False):

        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.mh.getMotor(1)
        self.rightMotor = self.mh.getMotor(2)

        self.verbose = verbose or debug
        self.debug = debug

        self.speed = 0.0
        self.angle = 0.0

        self.leftSpeed = 0.0
        self.rightSpeed = 0.0

        self.vmax = 1.0 / (1.0 + 0.5 * DiffDriveMotorInterface.__ratioAxelRadius)

        self.updatePWM()

    def updatePWM(self):

        v = self.speed * self.vmax;
        u = self.angle

        vl = v * (1.0 - u * 0.5 * DiffDriveMotorInterface.__ratioAxelRadius)
        vr = v * (1.0 + u * 0.5 * DiffDriveMotorInterface.__ratioAxelRadius)

        pwml = int(round(abs(vl) * (DiffDriveMotorInterface.__leftMaxSpeed - DiffDriveMotorInterface.__leftDeadZone) + DiffDriveMotorInterface.__leftDeadZone))
        pwmr = int(round(abs(vr) * (DiffDriveMotorInterface.__rightMaxSpeed - DiffDriveMotorInterface.__rightDeadZone) + DiffDriveMotorInterface.__rightDeadZone))

        if abs(vl) < 1e-4:
            leftMotorMode = Adafruit_MotorHAT.RELEASE
            pwml = 0;
        elif vl > 0:
            leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif vl < 0: 
            leftMotorMode = Adafruit_MotorHAT.BACKWARD

        if abs(vr) < 1e-4:
            rightMotorMode = Adafruit_MotorHAT.RELEASE
            pwmr = 0;
        elif vr > 0:
            rightMotorMode = Adafruit_MotorHAT.FORWARD
        elif vr < 0: 
            rightMotorMode = Adafruit_MotorHAT.BACKWARD

        if self.debug:
            print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl, vr, pwml, pwmr)

        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(leftMotorMode);
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(rightMotorMode);

    # Speed has to be in [-1, 1], sign determines bwd/fwd
    def setSpeed(self, speed):
        # Warning the user
        if abs(speed)>1:
            warnings.warn("Input speed has to be in [-1, 1]. Clamping it.")

        # Clamping speed value to [-1, 1]
        self.speed = numpy.clip(speed, -1, 1)

        if self.verbose:
            print "Speed is set to %5.3f" % self.speed

        self.updatePWM()
        


    def setSteerAngle(self, angle): 

        if abs(angle)>1:
            warnings.warn("Input angle has to be in [-1, 1]. Clamping it.")

        # Clamping speed value to [-1, 1]
        self.angle = numpy.clip(angle, -1, 1)

        if self.verbose:
            print "Steer angle is set to %5.3f" % self.angle

        self.updatePWM()


    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
        del self.mh
