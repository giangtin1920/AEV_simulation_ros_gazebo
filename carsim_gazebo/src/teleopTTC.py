#!/usr/bin/env python

from __future__ import print_function
from array import array

import rospy
import math 
import roslib; roslib.load_manifest('carsim_gazebo')
import sys, select, termios, tty

from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, UInt8
from rospy.numpy_msg import numpy_msg

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from carsim_gazebo.msg import ttcRadar_msg 

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
"""
moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1), # 'w':(1.1,1)
		'x':(.9,1), # 'x':(.9,1) 
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	settings = termios.tcgetattr(sys.stdin)
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def laserTTCCallback(msg):
	global disObj
	global posObj
	global posX
	global posY
	global ttc
	
	minRange = 15
	tmpRange = 0
	tmpIdx = 0
	cnt = 0
	disObj = []
	posObj = []
	posX = []
	posY = []
	ttc = []
	# disObj.clear()
	# posObj.clear()
	# 100 sample into 3 regions
	
	for idx, ranges in enumerate(msg.ranges):
		if ranges > minRange:
			obj = 0
		else:
			obj = 1

		if ranges < minRange:
			tmpRange += ranges	
			tmpIdx += idx
			cnt += 1

		if obj == 0 and cnt > 0:
			disObj.append(tmpRange / cnt)
			posObj.append(tmpIdx / cnt)
			tmpIdx = 0
			tmpRange = 0
			cnt = 0

		if obj == 1 and cnt == len(msg.ranges):
			disObj.append(tmpRange / cnt)
			posObj.append(tmpIdx / cnt)
			tmpIdx = 0
			tmpRange = 0
			cnt = 0
	for i, pos in enumerate(posObj):
		posObj[i] = msg.angle_max + msg.angle_increment*pos

	for i, dis in enumerate(disObj):
		posX.append(math.cos(posObj[i]) * dis )
		posY.append(math.sin(posObj[i]) * dis )
		ttc.append(dis/vel_carsim)

	

	# print(msg.angle_min)
	# print(msg.angle_max) // pi/2
	# print(msg.angle_increment)
	# print(msg.time_increment)

	angleRegions = msg.angle_max/5


	global regions
	regions = {
		'right':  msg.angle_max + angleRegions * 3,
		'front':  angleRegions * 4,
		'left':   msg.angle_max + angleRegions * 7,
	}

def autoDrive(*args):

	threshold_dist = 10
	ttc_min = 5
	global speed
	global turn
	global state_state_key
	speed = rospy.get_param("~speed", 2.0)
	turn = rospy.get_param("~turn", 30)
	state_key = []
	state_description = []
	
	if len(posObj) == 0:
		state_description.append('no obstacle')
		state_key.append("i")

	else:

		for i, pos in enumerate(posObj):

			if pos > regions['right'] and pos < regions['left']:
				state_description.append('front')
				state_key.append("x") if ttc[i] < ttc_min else state_key.append("i")

			elif pos <= regions['right'] :
				state_description.append('right')
				state_key.append("u") if ttc[i] < ttc_min else state_key.append("u")

			elif pos >= regions['left']:
				state_description.append('left')
				state_key.append("o") if ttc[i] < ttc_min else state_key.append("o")

			else:
				state_description.append('unknown')

	
	state_description  = list(set(state_description))
	state_description.sort()
	print(state_description)

	state_key = list(set(state_key))
	state_key.sort()
	# print(state_key)


	if state_key == ['i']:
		key = "i"
	elif state_key == ['o']:
		key = "o"
	elif state_key == ['u']:
		key = "u"
	elif state_key == ['x']:
		key = "x"

	elif state_key == ['i', 'o']:
		key = "o"
	elif state_key == ['i', 'u']:
		key = "u"
	elif state_key == ['i', 'x']:
		key = "x"

	elif state_key == ['o', 'u']:
		key = "k"
	elif state_key == ['o', 'x']:
		key = "o"

	elif state_key == ['u', 'x']:
		key = "u"	
	else:
		key = "k"
		print("none case")

	# print("dis = ", dis)
	# print("vel = ", vel_carsim)
	# print("ttc = ", ttc)
	# print(key)
	
		
	settings = termios.tcgetattr(sys.stdin)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		# print(msg)
		# print(vels(speed,turn))
		# while(1):
		# key = getKey()
		if key in moveBindings.keys():
			x = moveBindings[key][0]
			y = moveBindings[key][1]
			z = moveBindings[key][2]
			th = moveBindings[key][3]
		elif key in speedBindings.keys():
			speed = speed * speedBindings[key][0]
			turn = turn * speedBindings[key][1]

			print(vels(speed,turn))
			if (status == 14):
				print(msg)
			status = (status + 1) % 15
		else:
			x = 0
			y = 0
			z = 0
			th = 0
		# if (key == '\x03'):

		if abs(speed) > 10:
			speed = 10
		if abs(turn) > 80:
			turn = 80

		twist = Twist()
		twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
		pubTTC.publish(twist)

		ttcRadar = ttcRadar_msg()
		ttcRadar.numObj = len(posObj)
		ttcRadar.IdObj
		ttcRadar.isApproach = [True]
		ttcRadar.alpha = posObj
		ttcRadar.posX = posX
		ttcRadar.posY = posY
		ttcRadar.dis = disObj
		ttcRadar.vel.append(vel_carsim)
		ttcRadar.ttc = ttc

		ttcRadar.msg_counter = 10
		ttcRadar.isObject = True

		pubRadar.publish(ttcRadar)


	except Exception as e:
		print(e)

	# finally:
	# 	twist = Twist()
	# 	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	# 	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	# 	pub.publish(twist)

	# 	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def jointStateTTCCallback(msg):
	# print(msg.velocity)

	for idx, name in enumerate(msg.name):
		if name == "right_wheel_hinge":
			right_wheel_hinge = msg.velocity[idx]
		if name == "left_wheel_hinge":
			left_wheel_hinge = msg.velocity[idx]

	# print(right_wheel_hinge*0.3, left_wheel_hinge*0.3)
	global vel_carsim 
	vel_carsim = 0.3*(right_wheel_hinge+left_wheel_hinge)/2
	if vel_carsim == 0:
		vel_carsim = 0.001

def main():
	global pubTTC
	global pubRadar
	rospy.init_node('reading_laser')

	pubTTC = rospy.Publisher('/carsim1/cmd_vel', Twist, queue_size = 1)
	pubRadar = rospy.Publisher('ttcRadar_Data', ttcRadar_msg, queue_size = 1)

	subTTC = rospy.Subscriber('/carsim1/laser/scan', LaserScan, laserTTCCallback)
	velTTC = rospy.Subscriber('carsimTTC/joint_states', JointState, jointStateTTCCallback)
	ctr = rospy.Timer(rospy.Duration(0.1), autoDrive)
	rospy.spin()

if __name__ == '__main__':
	main()





	# if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
	# 	state_description = 'case 1 - no obstacle'
	# 	ttc = 15
	# 	dis = 15
	# 	global speed; speed = rospy.get_param("~speed", 2.0)
	# 	global turn; turn = rospy.get_param("~turn", 30)
	# 	global key; key = "i"

	# elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
	# 	state_description = 'case 7 - front and left and right'
	# 	dis = min(regions['front'], regions['left'], regions['right'])
	# 	ttc = dis/abs(vel_carsim)
	# 	if ttc < 2:
	# 		key = "x" 

	# elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
	# 	state_description = 'case 2 - front'
	# 	dis = regions['front']
	# 	ttc = dis/abs(vel_carsim)
	# 	key = "x" if ttc < ttc_min else "i"

	# elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
	# 	state_description = 'case 3 - right'
	# 	dis = regions['right']
	# 	ttc = dis/abs(vel_carsim)
	# 	key = "u" if ttc < ttc_min else "u"

	# elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
	# 	state_description = 'case 4 - left'
	# 	dis = regions['left']
	# 	ttc = dis/abs(vel_carsim)
	# 	key = "o" if ttc < ttc_min else "o"

	# elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
	# 	state_description = 'case 5 - front and right'
	# 	dis = regions['right']
	# 	ttc = dis/abs(vel_carsim)
	# 	key = "u" if ttc < ttc_min else "u"

	# elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
	# 	state_description = 'case 6 - front and left'
	# 	dis = regions['left']
	# 	ttc = dis/abs(vel_carsim)
	# 	key = "o" if ttc < ttc_min else "o"

	# elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
	# 	state_description = 'case 8 - left and right'
	# 	key = "k"
	# else:
	# 	state_description = 'unknown case'

	# # print(state_description)
	# # print("dis = ", dis)
	# # print("vel = ", vel_carsim)
	# # print("ttc = ", ttc)
	# # print(key)