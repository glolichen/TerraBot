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

class Sensors:
    time = 0
    light_level = 0
    moisture = 0
    humidity = 0
    temperature = 0
    weight = 0
    water_level = 0
    current = 0
    energy = 0
    light_level_raw = [0,0]
    moisture_raw = [0,0]
    humidity_raw = [0,0]
    temperature_raw = [0,0]
    weight_raw = [0, 0]

def init_sensors():
    global sensorsG
    sensorsG.time = rospy.get_time()

def init_ros():
    global led_pub, wpump_pub, fan_pub, ping_pub, camera_pub, speedup_pub, freq_pub, sensorsG
    
    rospy.set_param("use_sim_time", False)
    rospy.init_node("interactive_agent", anonymous = True)

    led_pub = rospy.Publisher("led_input", actuator_types['led'],
                              latch = True, queue_size = 1)
    wpump_pub = rospy.Publisher("wpump_input", actuator_types['wpump'],
                                latch = True, queue_size = 1)
    fan_pub = rospy.Publisher("fan_input", actuator_types['fan'],
                              latch = True, queue_size = 1)

    ping_pub = rospy.Publisher("ping", Bool, latch = True, queue_size = 1)
    camera_pub = rospy.Publisher("camera", actuator_types['cam'],
                                 latch = True, queue_size = 1)
    speedup_pub = rospy.Publisher("speedup", Int32, latch = True, queue_size = 1)
    freq_pub = rospy.Publisher("freq_input", actuator_types['freq'],
                               latch=True, queue_size=1)

    rospy.Subscriber("smoist_output", sensor_types['smoist'],
                     moisture_reaction, sensorsG)
    rospy.Subscriber("light_output", sensor_types['light'],
                     light_reaction, sensorsG)
    rospy.Subscriber("level_output", sensor_types['level'],
                     level_reaction, sensorsG)
    rospy.Subscriber("temp_output", sensor_types['temp'],
                     temp_reaction, sensorsG)
    rospy.Subscriber("humid_output", sensor_types['humid'],
                     humid_reaction, sensorsG)
    rospy.Subscriber("weight_output", sensor_types['weight'],
                     weight_reaction, sensorsG)
    rospy.Subscriber("cur_output", sensor_types['cur'],
                     power_reaction, sensorsG)

def moisture_reaction(data, sensorsG):
    sensorsG.moisture = (data.data[0] + data.data[1])/2.0
    sensorsG.moisture_raw = data.data

def humid_reaction(data, sensorsG):
    sensorsG.humidity = (data.data[0] + data.data[1])/2.0
    sensorsG.humidity_raw = data.data

def weight_reaction(data, sensorsG):
    sensorsG.weight = (data.data[0] + data.data[1])/2
    sensorsG.weight_raw = data.data

def temp_reaction(data, sensorsG):
    sensorsG.temperature = (data.data[0] + data.data[1])/2.0
    sensorsG.temperature_raw = data.data

def light_reaction(data, sensorsG):
    sensorsG.light_level = (data.data[0] + data.data[1])/2.0
    sensorsG.light_level_raw = data.data

def level_reaction(data, sensorsG):
    sensorsG.water_level = data.data

def power_reaction(data, sensorsG):
    sensorsG.current = -data.data[0]
    sensorsG.energy = -data.data[1]

def get_light_level():
    return sensorsG.light_level
def get_light_level_raw():
    return sensorsG.light_level_raw

def get_temperature():
    return sensorsG.temperature
def get_temperature_raw():
    return sensorsG.temperature_raw

def get_humidity():
    return sensorsG.humidity
def get_humidity_raw():
    return sensorsG.humidity_raw

def get_weight():
    return sensorsG.weight
def get_weight_raw():
    return sensorsG.weight_raw

def set_led(val: Int32):
    led_pub.publish(val)
def set_pump(val: Bool):
    wpump_pub.publish(val)
def set_fan(val: Bool):
    fan_pub.publish(val)
def send_camera(val: String):
    camera_pub.publish(val)
def set_speedup(val: Int32):
    speedup_pub.publish(val)
def set_frequency(val: String):
    freq_pub.publish(val)
def ping():
    ping_pub.publish(True)

sensorsG = Sensors()
    
init_ros()
init_sensors()

while rospy.get_time() == 0:
    rospy.sleep(0.1)
rospy.sleep(2) # Give a chance for the initial sensor values to be read
