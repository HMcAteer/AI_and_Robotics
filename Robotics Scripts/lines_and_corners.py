#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from laser_feature_extraction.msg import LineMsg, CornerMsg
##provided code
###############################################################
# Make an rviz line list given a list of Line objects
def buildRvizLineList(lines):

    line_list = Marker()
    line_list.header.frame_id = 'base_scan'
    line_list.header.stamp = rospy.Time(0)
    line_list.ns = ''

    line_list.id = 0
    line_list.type = 5
    line_list.action = 0
    
    line_list.scale.x = 0.02

    line_list.color.g = 1.0
    line_list.color.a = 1.0

    # Add the line endpoints to list of points
    for l in lines:
        line_list.points.append(l.p_a)
        line_list.points.append(l.p_b)
    
    return line_list

def buildRvizCorners(corners):

    pointMarker = Marker()
    pointMarker.header.frame_id = 'base_scan'
    pointMarker.header.stamp = rospy.Time(0)
    pointMarker.ns = ''

    pointMarker.id = 10
    pointMarker.type = 8
    pointMarker.action = 0
    
    pointMarker.scale.x = 0.2
    pointMarker.scale.y = 0.2
    pointMarker.scale.z = 0.2

    pointMarker.color.b = 1.0
    pointMarker.color.a = 1.0
    pointMarker.colors.append(pointMarker.color)

    
    for c in corners:
        pointMarker.points.append(c.p)

    #pub_rviz.publish(pointMarker)
    return pointMarker
##############################################################
##End of provided code
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

def laserScanCB(data):
	##print just to have something in here
	print(data)


def main():
	##initalize node
	rospy.init_node('lines_and_corners', anonymous=False)
	##create subscriber
	rospy.Subscriber('/scan', LaserScan, laserScanCB)
	##create publisher
	pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
	##making needed Point objects for testing
	test_Point_A = Point(0,1,0)
	test_Point_B = Point(1,0,0)
	test_Point_C = Point(1,1,0)
	test_Point_D = Point(2,2,0)
	test_Point_E = Point(1,2,0)
	test_Point_F = Point(2,1,0)
	##make 2 Line objects
	lineA = Line(0, 0, 0, test_Point_A, test_Point_B, 0)
	lineB = Line(0, 0, 0, test_Point_C, test_Point_D, 0)
	##put them in a list
	lineList = [lineA, lineB]
	##get the markers
	marker_line = buildRvizLineList(lineList)
	##make 2 corner objects
	cornerA = Corner(test_Point_E, 0, 0, 0, 0)
	cornerB = Corner(test_Point_F, 0, 0, 0, 0)
	##put them in a list
	cornerList = [cornerA, cornerB]
	##get the markers
	marker_corner = buildRvizCorners(cornerList)
	
	rate = rospy.Rate(10)
	##wait for one second
	rospy.sleep(1)	
	##publish the line and corner markers
	pub.publish(marker_line)
	pub.publish(marker_corner)

if __name__=='__main__':
	main()
