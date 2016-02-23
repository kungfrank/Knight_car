
# TODO: configuration
TOP = 2
BACK_LEFT = 0
BACK_RIGHT = 1
FRONT_LEFT = 3
FRONT_RIGHT = 4

# Do not change the constants: add your own colors or sequence

RED = [0.24, 0, 0]

s = 0.5
GREEN = [0, s, 0]
WHITE = [s, s, s]
BLUE = [0, 0, s]

GREEN2 = [0, 0.3, 0]


OFF = [0, 0, 0]

s = 0.5
WHITE = [s, s, s]

s = 0.8
WHITE2 = [s, s, s]

lightseq1 = [
	(0.5, {TOP: GREEN2, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),	
	(0.5, {TOP: OFF, BACK_LEFT:OFF, BACK_RIGHT:OFF, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),
]

lightseq2 = [
	(0.25, {TOP: GREEN2, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),	
	(0.25, {TOP: OFF, BACK_LEFT:OFF, BACK_RIGHT:OFF, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),
]

def cycle_LEDs(sequence):
	from time import sleep
	from .rgb_led import RGB_LED
	led = RGB_LED()

	for i in range(1000*1000):
		interval, config = sequence[i % len(sequence)]
		for k, col in config.items():
			print k, col
			led.setRGB(k, col)

		sleep(interval)

	del led
