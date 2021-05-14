#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist
import tf.transformations

##initalize needed global variables
odomData = Odometry()
twistData = Twist()
q = [0,0,0,0]

##odom call back function
def odomCB(data):
	global odomData
	odomData = data
	
	print(odomData)

def driveDistance(distance, publisher):##take in a specified distance and a publisher to move the robot
	##needed global variables
	global odomData
	global twistData
	pub = publisher
	
	##calculate the desired stop point
	stopPoint = odomData.pose.pose.position.x + distance
	##while the current position has not reached the stop point
	while odomData.pose.pose.position.x <= stopPoint:
		##set the linear.x velocity and publish to move the robot
		twistData.linear.x = 0.2
		pub.publish(twistData)
	##once we break out of the loop, stop moving because we have reached the desired point
	twistData.linear.x = 0.0
	pub.publish(twistData)
		
def turnTo(theta, publisher):##take in a specified orientation and a publisher to move the robot
	##needed global variables
	global odomData
	global twistData
	global q
	##set the goal and publisher
	goal = theta
	pub = publisher
	##boolean to check to see if we have arrived at the correct orientation
	arrived = False
	##upperBound and lowerBound to make sure we stop within the correct orientation values
	upperBound = goal + .02
	lowerBound = goal - .02

	##while arrived is false
	while not arrived:
		##save odomData.orientation into a list for the conversion function
		q[0] = odomData.pose.pose.orientation.x
		q[1] = odomData.pose.pose.orientation.y
		q[2] = odomData.pose.pose.orientation.z
		q[3] = odomData.pose.pose.orientation.w	
		##convert from quaternion to euler
		theta = tf.transformations.euler_from_quaternion(q)
		##if theta[2](z) has reached the desired orientation set arrived to true
		if theta[2] <= upperBound and theta[2] >= lowerBound:
			
			arrived = True
		##keep moving until about if statement has been satisfied. 
		twistData.angular.z = 0.2
		pub.publish(twistData)
	#once out of loop, stop moving
	twistData.angular.z = 0.0
	pub.publish(twistData)
def main():
	##initalize node
	rospy.init_node('drive_robot', anonymous=False)
	##initalize odom subscriber to get odometry data
	rospy.Subscriber('odom', Odometry, odomCB)
	##initalize publisher and rate to move the robot
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
	rate = rospy.Rate(30)
	##call both functions and then program ends so it does not run nonstop
	driveDistance(1, pub)##pass in to move one meter and publisher to move the robot
	turnTo(1.5, pub)##pass in to set orientation to 1.5 in terms of radians and publisher to move the robot
	


if __name__=='__main__':
	main()
