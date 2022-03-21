#!/usr/bin/env python

from __future__ import print_function

import rospy
import roslib; roslib.load_manifest('carsim_gazebo')
import sys, select, termios, tty

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

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
	# 100 sample into 3 regions
	sample = 100
	samRegions = sample/10
	minRange = 15
	global regions
	regions = {
		'right':  min(min(msg.ranges[0:samRegions*2]), minRange),
		'front':  min(min(msg.ranges[samRegions*2:samRegions*8]), minRange),
		'left':   min(min(msg.ranges[samRegions*8:samRegions*10-1]), minRange),
	}

def autoDrive(*args):
	threshold_dist = 10
	ttc_min = 5
		
	if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
		state_description = 'case 1 - no obstacle'
		ttc = 15
		dis = 15
		global speed; speed = rospy.get_param("~speed", 2.0)
		global turn; turn = rospy.get_param("~turn", 30)
		global key; key = "i"

	elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
		state_description = 'case 7 - front and left and right'
		dis = min(regions['front'], regions['left'], regions['right'])
		ttc = dis/abs(vel_carsim)
		if ttc < 2:
			key = "x" 

	elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
		state_description = 'case 2 - front'
		dis = regions['front']
		ttc = dis/abs(vel_carsim)
		key = "x" if ttc < ttc_min else "i"

	elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
		state_description = 'case 3 - right'
		dis = regions['right']
		ttc = dis/abs(vel_carsim)
		key = "u" if ttc < ttc_min else "u"

	elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
		state_description = 'case 4 - left'
		dis = regions['left']
		ttc = dis/abs(vel_carsim)
		key = "o" if ttc < ttc_min else "o"

	elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
		state_description = 'case 5 - front and right'
		dis = regions['right']
		ttc = dis/abs(vel_carsim)
		key = "u" if ttc < ttc_min else "u"

	elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
		state_description = 'case 6 - front and left'
		dis = regions['left']
		ttc = dis/abs(vel_carsim)
		key = "o" if ttc < ttc_min else "o"

	elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
		state_description = 'case 8 - left and right'
		key = "k"
	else:
		state_description = 'unknown case'

	print(state_description)
	print("dis = ", dis)
	print("vel = ", vel_carsim)
	print("ttc = ", ttc)
	print(key)
	
		
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
	rospy.init_node('reading_laser')

	pubTTC = rospy.Publisher('/carsim1/cmd_vel', Twist, queue_size = 1)
	subTTC = rospy.Subscriber('/carsim1/laser/scan', LaserScan, laserTTCCallback)
	velTTC = rospy.Subscriber('carsimTTC/joint_states', JointState, jointStateTTCCallback)
	ctr = rospy.Timer(rospy.Duration(0.1), autoDrive)
	rospy.spin()

if __name__ == '__main__':
	main()
