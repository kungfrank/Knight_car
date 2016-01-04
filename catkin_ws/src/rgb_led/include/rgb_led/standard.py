from rgb_led import RGB_LED
from time import sleep

from rgb_led import *
from duckietown_lights import *


led = RGB_LED()

led.setRGB(TOP, GREEN)
led.setRGB(BACK_LEFT, RED)
led.setRGB(BACK_RIGHT, RED)
led.setRGB(FRONT_LEFT, WHITE)
led.setRGB(FRONT_RIGHT, WHITE)

sleep(1000*1000)
del led