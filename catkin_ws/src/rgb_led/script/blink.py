#!/usr/bin/env python
from rgb_led import *
import sys

if len(sys.argv) <= 1:
	pattern = 'default'
else:
	pattern = sys.argv[1]


cycle_LEDs_named(pattern)
