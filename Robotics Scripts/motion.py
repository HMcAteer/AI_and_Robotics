#!/usr/bin/env python
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Pose
import rospy
import math
import tf.transformations
import numpy
import jacobians as jacobian
##global variables
pose = Pose()

cov = numpy.mat([[1,1,1], [1,1,1], [1,1,1]], float)

##robot specs
wheel_Cir = 0.2073
ticks_Per_Rev = 4096.0
rw = 0.1435
##initalize place holders
startTime = 0
endTime = 0
oldData = [0,0]
newData = [0,0]
##initalize global odomMsg
odomMsg = Odometry()
##Provided code
class State:

    def __init__(self, x, y, theta, vx, vy, vtheta):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.vy = vy
        self.vtheta = vtheta

def findDistanceBetweenAngles(a, b):
    '''
    Get the smallest orientation difference in range [-pi,pi] between two angles 
    Parameters:
        a (double): an angle
        b (double): an angle
    
    Returns:
        double: smallest orientation difference in range [-pi,pi]
    '''
    result = 0
    difference = b - a
    
    if difference > math.pi:
      difference = math.fmod(difference, math.pi)
      result = difference - math.pi

    elif(difference < -math.pi):
      result = difference + (2*math.pi)

    else:
      result = difference

    return result



def displaceAngle(a1, a2):
    '''
    Displace an orientation by an angle and stay within [-pi,pi] range
    Parameters:
        a1 (double): an angle
        a2 (double): an angle
    
    Returns:
        double: The resulting angle in range [-pi,pi] after displacing a1 by a2
    '''
    a2 = a2 % (2.0*math.pi)

    if a2 > math.pi:
        a2 = (a2 % math.pi) - math.pi

    return findDistanceBetweenAngles(-a1,a2)
##End of provided code

##Function to calculate u
def distTrav(newData, oldData):
	##needed global variables for calculations
	global wheel_Cir
	global ticks_Per_Rev
	##take in tick data
	curTickL = newData[0]
	oldTickL = oldData[0]
	curTickR = newData[1]
	oldTickR = oldData[1]
	##calculating deltaLeft and deltaRight
	deltaLeft = ((curTickL-oldTickL)/ticks_Per_Rev)*wheel_Cir
	deltaRight = ((curTickR-oldTickR)/ticks_Per_Rev)*wheel_Cir
	##store deltaLeft and deltaRight in u
	u = [deltaLeft, deltaRight]
	return u

def transitionModel(x, u):
	##needed global variables for calculations
	global rw
	newState = x
	deltaLeft = u[0]
	deltaRight = u[1]
	global startTime
	global endTime
	time = endTime - startTime
	
	##calculate deltatheta
	deltaTheta =((deltaRight - deltaLeft)/(2*rw))
	newState.theta = displaceAngle(newState.theta,deltaTheta)
	##calculate d(distance travelled by robot)
	d = ((deltaLeft + deltaRight)/2)
	##calculate the change in x and y 
	deltaX = d*math.cos(newState.theta)
	deltaY = d*math.sin(newState.theta)
	##update state with new positon
	newState.x = newState.x+deltaX
	newState.y = newState.y+deltaY
	##update velocity values
	newState.vx = deltaX/time
	newState.vy = deltaY/time
	newState.vtheta = ((newState.vx - newState.vy)/(2*rw))

	return  newState

def buildOdomMsg(state, odomMsg):
	##update odom position values
	odomMsg.pose.pose.position.x = state.x
	odomMsg.pose.pose.position.y = state.y
	odomMsg.pose.pose.position.z = 0
	##convert from euler to quaternion
	temp = tf.transformations.quaternion_from_euler(0, 0, state.theta) 
	##update odom orientation values using quaternion from above function
	odomMsg.pose.pose.orientation.x = temp[0]
	odomMsg.pose.pose.orientation.y = temp[1]
	odomMsg.pose.pose.orientation.z = temp[2]
	odomMsg.pose.pose.orientation.w = temp[3]
	##update twist values using velocity
	odomMsg.twist.twist.linear.x = state.vx
	odomMsg.twist.twist.linear.y = state.vy
	odomMsg.twist.twist.linear.z = 0.0
	
	odomMsg.twist.twist.angular.x = 0.0
	odomMsg.twist.twist.angular.y = 0.0
	odomMsg.twist.twist.angular.z = state.vtheta
	return odomMsg
##Call back function for encoder positions
def sensorCB(data):
	##needed global variables for calculations
	global oldData
	global newData
	global odomMsg
	global x
	global startTime
	global endTime
	global pose
	global cov
	##get start time
	startTime = rospy.get_time()
	##get end time
	endTime = rospy.get_time()
	##getting updated encoder 
	if oldData[0] == 0 and oldData[1] == 0:##checks for first pass through
		
		newData[0] = data.left_encoder
		newData[1] = data.right_encoder
		oldData[0] = data.left_encoder
		oldData[1] = data.right_encoder

	else:##every other pass through
		oldData[0] = newData[0]
		oldData[1] = newData[1]
		newData[0] = data.left_encoder
		newData[1] = data.right_encoder
	##calculate u using encoder
	u = distTrav(newData, oldData)
	##updating state
	
	##build the odomMsg
	buildOdomMsg(x, odomMsg)
	x = transitionModel(x,u)

	##update the pose from the newest state update
	pose.position.x = x.x
	pose.position.y = x.y
	pose.position.z = 0
	temp = tf.transformations.quaternion_from_euler(0, 0, x.theta) 
	pose.orientation.x = temp[0]
	pose.orientation.y = temp[1]
	pose.orientation.z = temp[2]
	pose.orientation.w = temp[3]
	cov = updateCov(pose, cov, u)
	print(pose)
	print(cov)
	
def updateCov(pose, cov, u):
	
	G = jacobian.getG(pose, u)
	V = jacobian.getV(pose, u)
	M = numpy.mat([[0.05*u[0],0],[0,0.05*u[1]]], float)
	newCov = ((G*cov*G.transpose()) + (V*M*V.transpose()))

	return newCov
def main():
	global x 
	x = State(0,0,0,0,0,0)	
	##initialize sensor node	
	rospy.init_node('sensor', anonymous=False)
	##initialize subscriber to retrieve encoder positions
	
	rospy.Subscriber('/sensor_state', SensorState, sensorCB)
	
	##initialize publisher to publish odomMsg to my_odom
	pub = rospy.Publisher('my_odom', Odometry, queue_size = 10)
	##set the rate
	rate = rospy.Rate(10)


	##loop to run indefinitely 
	while not rospy.is_shutdown():
		pub.publish(odomMsg)
		rate.sleep()


if __name__=='__main__':
	main()
