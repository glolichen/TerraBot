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

def init_ros():
	global ping_pub

time = 0
last_ping = 0

level = 0
pump = False
fan = False
pump_time = 4
last_water = 0
light_time = 32400
last_light = 0
fan_time = 10
last_fan = 0
init_ros()
rospy.sleep(2) # Give a chance for the initial sensor values to be read
while rospy.get_time() == 0: rospy.sleep(0.1) # Wait for clock to start up correctly
print("Connected and ready for interaction")

while not rospy.core.is_shutdown():
	time = rospy.get_time()

	# Ping every 3 minutes, twice as frequently as timeout in TerraBot
	if (time - last_ping) >= 180:
		wrapper.ping()

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

	### Check for input
	# if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
	# 	input2 = sys.stdin.readline()
	# 	print(input2)
	# 	if input2[0] == 'q':
	# 		quit()
	# 	else: 
	# 		try:
	# 			if input2[0] == 'p':
	# 				wpump_pub.publish(input2.find("on") > 0)
	# 			elif input2[0] == 'f':
	# 				fan_pub.publish(input2.find("on") > 0)
	# 			elif input2[0] == 'l':
	# 				#print(input[1:])
	# 				#id = ["on", "off"].index(input[1:].strip(" ")) + 1
	# 				#print(id)
	# 				#level = int([input[1:], 255, 0][id])
	# 				level = input2.split(" ")[1].strip("\n")
	# 				level = int([255, 0][["on", "off"].index(level)] if "o" in level else level)
	# 				led_pub.publish(level)
	# 			elif input2[0] == 'v':
	# 				print("Humidity is: %.1f" %(sensorsG.humidity))
	# 				print("Light Level is: %.1f" %(sensorsG.light_level))
	# 				print("Temperature is: %.1f" %(sensorsG.temperature))
	# 				print("Soil Moisture: %.1f" %(sensorsG.moisture))
	# 			else:
	# 				print("write an actual command / follow instructions")
	# 		except Exception:
	# 			print("command error / follow my expectations")
	rospy.sleep(1)
