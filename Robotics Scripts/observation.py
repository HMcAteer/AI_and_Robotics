#!/usr/bin/env python
import numpy
import rospy
import math
import jacobians as jacobian
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from laser_feature_extraction.msg import LineMsg, CornerMsg, DepthFeatures
from geometry_msgs.msg import Pose

pose = Pose()
cov = numpy.mat([[1,1,1], [1,1,1], [1,1,1]], float)
##create global publishers
##provided code
###############################################################
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

##############################################################
##End of provided code
##############################################################
##From Prep Work
class Line:
	##creating the fields
	coeffA = 0
	coeffB = 0
	coeffC = 0
	p_a = Point()
	p_b= Point()
	ID = 0
	msg = LineMsg()
	##setting the fields to the passed in parameters
	def __init__(self,coefficientA, coefficientB, coefficientC, point_a, point_b, iD):
		self.coeffA = coefficientA
		self.coeffB = coefficientB
		self.coeffC = coefficientC
		self.p_a = point_a
		self.p_b = point_b
		self.ID = iD

		##calculate deltaX and deltaY in order to check if slope may equal zero
		deltaY = self.p_b.y-self.p_a.y
		deltaX = self.p_b.x-self.p_a.x
		##calculate length using distance formula
		self.length = math.sqrt(deltaX**2+deltaY**2)
		##make sure it will not be dividing my zero, if so set slope to 0
		if deltaY == 0 or deltaX == 0:
			self.slope = 0
		else:
	##calculate slope
			self.slope = deltaY/deltaX
	##create LineMsg using fields
		self.msg = LineMsg(self.coeffA, self.coeffB, self.coeffC, self.p_a, self.p_b, self.ID)

class Corner:
	##creating the fields
	p = Point()
	psi = 0
	ID = 0
	l_a = LineMsg()
	l_b = LineMsg()
	msg = CornerMsg()
	##setting the fields to the passed in parameters
	def __init__(self, point, psi, iD, line_a, line_b):
		self.p = point
		self.psi = psi
		self.ID = iD
		self.l_a = line_a
		self.l_b = line_b
	##create CornerMsg using fields
		self.msg = CornerMsg(self.p, self.psi, self.ID, self.l_a, self.l_b)
##############################################################
##End of Prep Work
def getLineBetweenPoints(point_A, point_B):
	##get the x and y points for line equation calculation
	x_one = point_A.x
	x_two = point_B.x
	y_one = point_A.y
	y_two = point_B.y

	##calculate slope
	if ((y_two-y_one == 0) or (x_two-x_one == 0)):
		slope = 0
	else:
		slope = (y_two-y_one)/(x_two-x_one)
	
	
	##calcualte cefficent values for Ax+By+c=0 from slope intercept form
	##A is always slope 
	coeffA = slope
	##B is always -1
	coeffB = -1
	##C is the y-intercept (b) value in slope intercept form
	coeffC = ((slope*(-x_one))+y_one)
	##normalize
	normal = math.sqrt((coeffA**2) + (coeffB**2))
	coeffA = coeffA/normal
	coeffB = coeffB/normal
	coeffC = coeffC/normal
	##create the line object
	line = Line(coeffA, coeffB, coeffC, point_A, point_B, 0)
	##return the line object	
	return line

def getDistanceToLine(point, line):
	##get x and y values from passed in point object
	x = point.x
	y = point.y
	##d = Ax + By + C, where d is the distance
	d = (line.coeffA*x) + (line.coeffB*y) + line.coeffC
	return d

def getLine(points, firstPass):
	##if this is the first call of getLine then we have to first calculate which points are the farthest away
	if firstPass is True:
		highD = 0
		tempD = 0
		tempIndex = 0
		index_point_a = 0
		index_point_b = 0
		for i,p in enumerate(points):
			for x,l in enumerate(points):

				currD = math.sqrt((p.x-l.x)**2+(p.y-l.y)**2)##calculate the distance
				if highD < currD:##if distance is higher
					highD = currD##save new highest distance
					index_point_a = i##save the index positions
					index_point_b = x
		line = getLineBetweenPoints(points[index_point_a], points[index_point_b])
		##enumerate through all the points
		for i,p in enumerate(points):
			##calculate the distance from the line for each point
			d = abs(getDistanceToLine(p, line))
			##if the distance is greater and the is greater then the current highest distance
			if d > .1 and d > tempD:
				tempD = d ##save as the new highest
				tempIndex = i ##save the index position of the point
		if tempD > 0:
			return -1, tempIndex
		##otherwise return the success and the line
		if tempD == 0:
			return 1, line
			 
	if firstPass is False:
		tempD = 0
		tempIndex = 0
		##get the first and last point in the list
		first_point = points[0]
		last_point = points[-1]
		##create a line object from the first and last point	
		line = getLineBetweenPoints(first_point, last_point)
		##enumerate through all the points
		for i,p in enumerate(points):
			##calculate the distance from the line for each point
			d = abs(getDistanceToLine(p, line))
			##if the distance is greater and the is greater then the current highest distance
			if d > .1 and d > tempD: 
				tempIndex = i ##save the index position of the point
				tempD = d ##save as the new highest		
		if tempD > 0:
			return -1, tempIndex
		##otherwise return the success and the line
		if tempD == 0:
			return 1, line
		
def getAllLines(points):
	##create needed empty lists
	toProcess = []##will be processed to find the correct lines
	lineList = []##list of acceptabe lines
	##bool to signal if it is the first getLine
	firstPass = True
	## first call of getLine
	success, step = getLine(points, firstPass)
	##no longer the first pass so set firstPass to False
	firstPass = False
	##checks to see if line was within acceptable threshold
	if success == -1:##if not
		##split the list of points using List Slicing
		##append the list to the toProcess list
		p_a = points[0:step]
		p_b = points[step:]
		toProcess.append(p_a)##grab the beginning to the highest distance
		toProcess.append(p_b)##grab from the highest distance to the end of the list	
	if success == 1:##if line was acceptable based on the threshold
		lineList.append(step)##append line to the fininished lineList

	##start the loop to find all of the acceptable line objects
	while(len(toProcess) > 0):
		
		if len(toProcess[0]) > 2:
			
			currPoints = toProcess[0]##take the first list of points to find the line
			
			##start find line segments
			success_loop, step_loop = getLine(currPoints, firstPass)
			if success_loop == -1:##if line is not acceptable
				##split the list of points using List Slicing
				##append the list to the toProcess list
				p_a = currPoints[0:step_loop]
				p_b = currPoints[step_loop:]
				toProcess.append(p_a)##grab the beginning to the highest distance
				toProcess.append(p_b)##grab from the highest distance to the end of the list
				
				del toProcess[0]##delete them from the list so we do not repeat nonstop
				
			if success_loop == 1:##if line was acceptable based on the threshold
				del toProcess[0]##delete them from the list so we do not repeat nonstop
				line = step_loop
				lineList.append(line)##append line to the fininished lineList
		else:
			del toProcess[0]
	return lineList	
		
def getCornersFromLines(lines):
	corners = []
	
	for i_one, line_one in enumerate(lines):
		for i_two, line_two in enumerate(lines):
			
			nume = line_two.slope - line_one.slope
			denom = 1 + (line_two.slope * line_one.slope)
			placeHolder = nume/denom
			deltaTheta = math.degrees(math.atan(placeHolder))
			first_endPoints = [line_one.p_a, line_one.p_b]	
			second_endPoints = [line_two.p_a, line_two.p_b]
			
			if deltaTheta > 45 and deltaTheta < 120:
				for firstPoint in first_endPoints:
					for secondPoint in second_endPoints:
						d = math.sqrt((secondPoint.x-firstPoint.x)**2 + (secondPoint.y - firstPoint.y)**2)
						if d < .3:
							midX = (firstPoint.x + secondPoint.x)/2
							midY = (firstPoint.y + secondPoint.y)/2
							midPoint = Point(midX, midY, 0)
							corner = Corner(midPoint, deltaTheta, 0, line_one, line_two)
							corners.append(corner)
	return corners
		

def updateCov(cMsg, cov, pose):
	
	q = math.sqrt((cMsg.p.x-pose.position.x)**2+(cMsg.p.y-pose.position.y)**2)
	Q = numpy.mat([[0.05,0,0],[0,0.05,0], [0,0,0.05]], float)
	H = jacobian.getH(cMsg,pose, q)
	newCov = (((H*cov)*H.transpose()) + (Q))

	return newCov
		
def laserScanCB(data):
	##do a cov for each corner instead of one cov
	global cov
	global pose
	pointList = []
	linesMsg = []
	cornersMsg = []
	##Getting Cartesian Coordinates
	##enumerate data.ranges to get the distances to each point
	for i,d in enumerate(data.ranges):
		##check to make sure the distances are withing the acceptable ranges
		if d > data.range_min and d < data.range_max:
			tempAngle = i*data.angle_increment##calculate the tempAngle in order to use the displace angle
			polarAngle = displaceAngle(data.angle_min, tempAngle) ##use displace angle to get the polarAngle
			cartX = d*math.cos(polarAngle)##calculate the cartesian X coordinate 
			cartY = d*math.sin(polarAngle)##calculate the cartesian Y coordinate
			pointList.append(Point(cartX, cartY, 0))##Append the newly created Point obj to the pointList


	lines = getAllLines(pointList)
	corners = getCornersFromLines(lines)
	for l in lines:
		linesMsg.append(l.msg)
	for c in range(len(corners)-1,0,-1):
		##try going backwards
		#print(c.msg.p)
		cov = updateCov(corners[c], cov, pose)
		print(cov)
		#cornersMsg.append(c.msg)
	
	depthFeatures = DepthFeatures('Header', linesMsg, cornersMsg)
	
def main():
	print("In Main")
	global pose
	pose.orientation.x = 0
	pose.orientation.y = 0
	pose.orientation.z = 0
	pose.orientation.w = 1
	##initalize node
	rospy.init_node('lines_and_corners', anonymous=False)
	##create subscriber
	rospy.Subscriber('/scan', LaserScan, laserScanCB)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep
if __name__=='__main__':
	main()
