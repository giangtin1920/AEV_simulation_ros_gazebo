#!/usr/bin/env python

from __future__ import print_function

import rospy
import roslib; roslib.load_manifest('carsim_gazebo')
import sys, select, termios, tty

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

is_run = True

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
		'w':(1.2,1), # 'w':(1.1,1)
		'x':(.8,1), # 'x':(.9,1) 
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	settings = termios.tcgetattr(sys.stdin)
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [])
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def controlCarsim2(msg):
	
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0
	global is_run
	if is_run:
		is_run = False
		global speed
		global turn
		speed = 2
		turn = 40
	
	try:
		# print(msg)
		# print(vels(speed,turn))
		# while(1):
		keyObj = getKey()
		
		if keyObj in moveBindings.keys():
			x = moveBindings[keyObj][0]
			y = moveBindings[keyObj][1]
			z = moveBindings[keyObj][2]
			th = moveBindings[keyObj][3]
		elif keyObj in speedBindings.keys():
			speed = speed * speedBindings[keyObj][0]
			turn = turn * speedBindings[keyObj][1]

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
		pubObj.publish(twist)

	except Exception as e:
		print(e)


def jointStateObjCallback(msg):
	# print(msg.velocity)
	
	for idx, name in enumerate(msg.name):
		if name == "right_wheel_hinge":
			right_wheel_hinge = msg.velocity[idx]
		if name == "left_wheel_hinge":
			left_wheel_hinge = msg.velocity[idx]

	# print(right_wheel_hinge*0.3, left_wheel_hinge*0.3)
	global velObj
	velObj = 0.3*(right_wheel_hinge+left_wheel_hinge)/2
	if velObj == 0:
		velObj = 0.001

	print("\r\n", velObj, "\r\n")



def main():
	global pubObj
	
	rospy.init_node('control_carsim2')
	pubObj = rospy.Publisher('/carsim2/cmd_vel', Twist, queue_size = 1)
	subObj = rospy.Subscriber('/carsim2/laser/scan', LaserScan, controlCarsim2)
	velObj = rospy.Subscriber('/carsimObj/joint_states', JointState, jointStateObjCallback)
	# ctrObj = rospy.Timer(rospy.Duration(0.1), manDrive)

	rospy.spin()

if __name__ == '__main__':
	main()

