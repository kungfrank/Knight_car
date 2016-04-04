# -*- coding: utf-8 -*-
"""
Created on Sun Apr  3 19:50:11 2016

@author: alex
"""

V = Vector( 0.994964746981 , -0.0280961326221 ,  0.0277208000733  )

w = 0.0921266348012

Q = Quaternion( V , w )

a = Q.toAngleAxis()

C1 = euler2RotationMatrix(180,0,0)
C2 = euler2RotationMatrix(0,180,0)

A1 = AngleAxis( rad = np.pi , axis = Vector(1,0,0) )
A2 = AngleAxis( rad = np.pi , axis = Vector(0,1,0) )

Q1 = A1.toQuaternion()
Q2 = A2.toQuaternion()