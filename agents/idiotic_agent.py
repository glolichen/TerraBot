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

def clamp(min, max, num):
    if num < min:
        return min
    if num > max:
        return max
    return num

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

parser = argparse.ArgumentParser(description = "Interactive Agent")
parser.add_argument('-l', '--log', action = 'store_true',
                    help="print sensor values")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
args = parser.parse_args()

sensorsG = Sensors()
is_logging = args.log
use_simulator = args.sim

def init_sensors():
    global sensorsG
    sensorsG.time = rospy.get_time()

### ROS-related stuff
### Set up publishers, subscribers, and message handlers

def init_ros ():
    global led_pub, wpump_pub, fan_pub, ping_pub, camera_pub, speedup_pub, freq_pub, sensorsG

    if use_simulator: rospy.set_param("use_sim_time", True)
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
    if is_logging: print("    Moisture: %d %d" %(data.data[0], data.data[1]))

def humid_reaction(data, sensorsG):
    sensorsG.humidity = (data.data[0] + data.data[1])/2.0
    sensorsG.humidity_raw = data.data
    if is_logging: print("    Humidity: %d %d" %(data.data[0], data.data[1]))

def weight_reaction(data, sensorsG):
    # Each weight sensor holds half the weight of the pan
    sensorsG.weight = (data.data[0] + data.data[1])/2
    sensorsG.weight_raw = data.data
    if is_logging: print("    Weight: %d %d" %(data.data[0], data.data[1]))

def temp_reaction(data, sensorsG):
    sensorsG.temperature = (data.data[0] + data.data[1])/2.0
    sensorsG.temperature_raw = data.data
    if is_logging: print("    Temperature: %d %d" %(data.data[0], data.data[1]))

def light_reaction(data, sensorsG):
    sensorsG.light_level = (data.data[0] + data.data[1])/2.0
    sensorsG.light_level_raw = data.data
    if is_logging: print("    Lights: %d %d" %(data.data[0], data.data[1]))

def level_reaction(data, sensorsG):
    sensorsG.water_level = data.data
    if is_logging: print("    Level: %.2f" %data.data)

def power_reaction(data, sensorsG):
    sensorsG.current = -data.data[0]
    sensorsG.energy = -data.data[1]
    if is_logging:
        print("    Current: %f Energy: %f" %(sensorsG.current, sensorsG.energy))

def cam_reaction(data):
    print ("picture taken\t" + data.data)

last_ping = 0
def ping():
    global last_ping
    #print("PING! %s" %clock_time(sensorsG.time))
    last_ping = sensorsG.time
    ping_pub.publish(True)

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
init_sensors()
rospy.sleep(2) # Give a chance for the initial sensor values to be read
while rospy.get_time() == 0: rospy.sleep(0.1) # Wait for clock to start up correctly
print("Connected and ready for interaction")

while not rospy.core.is_shutdown():
    sensorsG.time = rospy.get_time()

    # Ping every 3 minutes, twice as frequently as timeout in TerraBot
    if (sensorsG.time - last_ping) >= 180: ping()

    light = sensorsG.light_level
    temp = sensorsG.temperature
    humid = sensorsG.humidity
    moist = sensorsG.moisture
    weight = sensorsG.weight
    reservoir = sensorsG.water_level

    #fan = humid > humidity_avg

    if((sensorsG.time - last_water) > 43200): #pump schedule 3 seconds every 12 hours
        pump_time = 3
        pump = True
        last_water = sensorsG.time
    elif(pump_time==0):
        pump = False

    if((sensorsG.time - last_light) > 86400): #light schedule 9 hours for every 24 hours
        light_time = 32400
        level = 255
        last_light = sensorsG.time
    elif(light_time==0):
        level = 0

    if((sensorsG.time - last_fan) > 7200): #fan schedule 10 seconds for every 2 hours
        fan_time = 10
        fan = True
        last_fan = sensorsG.time
    elif(fan_time==0):
        fan = False


    pump_time-=1
    wpump_pub.publish(pump)
    light_time-=1
    led_pub.publish(level)
    fan_time-=1
    fan_pub.publish(fan)

    print(pump,fan,level,weight)

    ### Check for input
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input2 = sys.stdin.readline()
        print(input2)
        if input2[0] == 'q':
            quit()
        else: 
            try:
                if input2[0] == 'p':
                    wpump_pub.publish(input2.find("on") > 0)
                elif input2[0] == 'f':
                    fan_pub.publish(input2.find("on") > 0)
                elif input2[0] == 'l':
                    #print(input[1:])
                    #id = ["on", "off"].index(input[1:].strip(" ")) + 1
                    #print(id)
                    #level = int([input[1:], 255, 0][id])
                    level = input2.split(" ")[1].strip("\n")
                    level = int([255, 0][["on", "off"].index(level)] if "o" in level else level)
                    led_pub.publish(level)
                elif input2[0] == 'v':
                    print("Humidity is: %.1f" %(sensorsG.humidity))
                    print("Light Level is: %.1f" %(sensorsG.light_level))
                    print("Temperature is: %.1f" %(sensorsG.temperature))
                    print("Soil Moisture: %.1f" %(sensorsG.moisture))
                else:
                    print("write an actual command / follow instructions")
            except Exception:
                print("command error / follow my expectations")
    rospy.sleep(1)
