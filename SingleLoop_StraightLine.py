#!/usr/bin/env/ python

import rospy
import tf
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, Pose, Quaternion
from math import cos,sin,pi,sqrt,atan2
import time


quaternion = yaw = euler = theta = x = y = 0
Goal = Pose()
def callback(msg):
	global quaternion
	global euler
	global theta
	global yaw
	global x
	global y
	x = msg.pose.pose.position.x 
	y = msg.pose.pose.position.y
	quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]
	theta = 180/pi*yaw
	if theta < 0:
	  theta = theta + 360

def trial(): 
  
	rospy.init_node('single_loop', anonymous=True)
     
	pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
	publishGoal = rospy.Publisher('Target_space', Pose, queue_size = 10)

	rate = rospy.Rate(10)

	rospy.Subscriber('odom',Odometry,callback)

	t0 = time.clock()

	rospy.sleep(3)

	move_fwd = Twist()
	move_fwd.linear.x = 0.5

	turn = Twist()

	turn_r = Twist()

	t1 = time.clock()
	phi = t1-t0
	m = 2
	r = sqrt(1+m*m)

	x1 =  x + phi/r
	y1=  y + phi*m/r

	print 'The distance from target is:', phi

	while phi > 0.01:
		t1 = time.clock()
		phi = t1 -t0
		x1 =  x + phi/r
		y1=  y + phi*m/r

		Goal.position.x = x1
		Goal.position.y = y1
		publishGoal.publish(Goal)

		theta1 = (atan2((y1-y),(x1-x)))*(180/pi)
		if theta1 < 0:
			theta1 = theta1 + 360
		
		rotate = theta1 - theta
		if rotate < 0:
			rotate = rotate + 360
	
		if rotate < 180:
			turn_r.angular.z = rotate/180
			turn_r.linear.x = 0.25*phi

			pub.publish(turn_r)

		else:
			turn.angular.z = -((360-rotate)/180)
			turn.linear.x = 0.25*phi

			pub.publish(turn)
	

		rate.sleep()


	rate.sleep()
	
if __name__ == "__main__":
    try:
      trial()
    except rospy.ROSInterruptException:
      pass
