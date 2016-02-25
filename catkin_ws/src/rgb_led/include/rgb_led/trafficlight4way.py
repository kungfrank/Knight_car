#!/usr/bin/env python
from rgb_led import *
from time import sleep

from duckietown_lights import *


led = RGB_LED()

interval = 5
configs = [
	{TOP: GREEN, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:RED,FRONT_RIGHT:RED},	
	{TOP: RED, BACK_LEFT:RED, BACK_RIGHT:GREEN, FRONT_LEFT:RED,FRONT_RIGHT:RED},	
	{TOP: RED, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:GREEN,FRONT_RIGHT:RED},
{TOP: RED, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT: RED,FRONT_RIGHT:GREEN},	
]

for i in range(1000*1000):
	config = configs[i % len(configs)]
	for k, col in config.items():
		print k, col
		led.setRGB(k, col)

	sleep(interval)

del led
