#!/usr/bin/env python
from rgb_led import *
from time import sleep

from duckietown_lights import *


led = RGB_LED()

interval = 0.5
configs = [
	{TOP: GREEN, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:WHITE,FRONT_RIGHT:WHITE},	
	{TOP: OFF, BACK_LEFT:OFF, BACK_RIGHT:OFF, FRONT_LEFT:OFF,FRONT_RIGHT:OFF},
]

for i in range(1000*1000):
	config = configs[i % len(configs)]
	for k, col in config.items():
		print k, col
		led.setRGB(k, col)

	sleep(interval)

del led
