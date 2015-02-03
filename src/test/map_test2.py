#!/usr/bin/env python

import roslib; roslib.load_manifest('rviz')
from nav_msgs.msg import OccupancyGrid
import rospy, numpy
import math

publisher101 = rospy.Publisher('map_101_values', OccupancyGrid)
publisher256 = rospy.Publisher('map_256_values', OccupancyGrid)

rospy.init_node('map_test2')

grid = OccupancyGrid()
t = 0

while not rospy.is_shutdown():
    w = h = valcount = 256

    grid.header.frame_id = "/map"
    grid.header.stamp = rospy.Time.now()
    grid.info.map_load_time = rospy.Time.now()
    grid.info.resolution = 0.1
    grid.info.width = w
    grid.info.height = h
    grid.info.origin.position.x = -w * grid.info.resolution / 2.0
    grid.info.origin.position.y = -h * grid.info.resolution / 2.0
    grid.info.origin.orientation.w = 1.0

    grid.data = [0] * w * h

    for x in xrange(0, w):
        for y in xrange(0, h):
            # Allowed range is -128 to +127
            grid.data[y*h + x] = (x + t) % valcount - 128

    publisher256.publish( grid )
    
    for x in xrange(0, w):
        for y in xrange(0, h):
            # Allowed range is 0 to 100
            grid.data[y*h + x] = ((x + t) % valcount) * 100 / 256

    publisher101.publish( grid )
    
    rospy.sleep(1.0 / 30.0)
    #t += 1
