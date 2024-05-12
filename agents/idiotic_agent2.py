#!/usr/bin/env python

import rospy, sys, select, os
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import argparse
import limits
from datetime import datetime
sys.path.insert(0, os.getcwd()[:os.getcwd().find('TerraBot')]+'TerraBot/lib')
from terrabot_utils import clock_time
from freqmsg import tomsg
from topic_def import sensor_types, actuator_types
import traceback
import wrapper

light_optimal_low, light_optimal_high = limits.optimal["light_level"]
water_optimal_low, water_optimal_high = limits.optimal["water_level"]
moisture_optimal_low, moisture_optimal_high = limits.optimal["moisture"]
humidity_optimal_low, humidity_optimal_high = limits.optimal["humidity"]
temp_optimal_low, temp_optimal_high = limits.optimal["temperature"]

light_avg = (light_optimal_low + light_optimal_high) / 2
water_avg = (water_optimal_low + water_optimal_high) / 2
moisture_avg = (moisture_optimal_low + moisture_optimal_high) / 2
humidity_avg = (humidity_optimal_low + humidity_optimal_high) / 2
temp_avg = (temp_optimal_low + temp_optimal_high) / 2

### ROS-related stuff
### Set up publishers, subscribers, and message handlers

time, last_ping = 0, 0

level = 0
pump = False
fan = False
pump_time = 4
last_water = 0
light_time = 32400
last_light = 0
fan_time = 10
last_fan = 0
print("Connected and ready for interaction")

while not rospy.core.is_shutdown():
	time = rospy.get_time()

	# Ping every 3 minutes, twice as frequently as timeout in TerraBot
	if (time - last_ping) >= 180:
		wrapper.ping()
		last_ping = time

	if((time - last_water) > 43200): #pump schedule 3 seconds every 12 hours
		pump_time = 5
		pump = True
		last_water = time
	elif(pump_time==0):
		pump = False

	if((time - last_light) > 86400): #light schedule 9 hours for every 24 hours
		light_time = 32400
		level = 255
		last_light = time
	elif(light_time==0):
		level = 0

	if((time - last_fan) > 7200): #fan schedule 10 seconds for every 2 hours
		fan_time = 10
		fan = True
		last_fan = time
	elif(fan_time==0):
		fan = False


	pump_time-=1
	wrapper.set_pump(pump)
	light_time-=1
	wrapper.set_led(level)
	fan_time-=1
	wrapper.set_fan(fan)

	print(pump, fan, level, wrapper.get_weight(), wrapper.get_light_level())

	if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
		input2 = sys.stdin.readline()
		if input2[0] == 'q':
			quit()
		elif input2[0] == 's':
			wrapper.set_speedup(int(input2[1:]))
		else:
			print("problem")

	rospy.sleep(1)
