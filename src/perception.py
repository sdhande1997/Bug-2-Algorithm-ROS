#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import math
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

zeros_list = np.zeros((361, 2))
degree = np.linspace(-1*np.pi/2, np.pi/2, 361)
x_coord = 1
y_coord = 1
thresh = 3
horizon = None
coord_X = []                     #To store the final X points
coord_Y = []
efficiency = []
point_threshold = 3


def get_Ransac(laser):
    global sinx, cosx, degree, run_rate, zeros_list
    global x_coord, y_coord, coord_X, coord_Y, k, scan_time
    global coord_X, coord_Y,a2, b2, a3, b3, lists
    marker = Marker()
    pub = rospy.Publisher("visualize_ransac", Marker, queue_size=10)
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.ns = "perception"
    marker.action = Marker.ADD
    marker.type = Marker.LINE_STRIP
    
    p1x = 0
    p1y = 0
    p2x = 0
    p2y = 0
    
    horizon = laser.ranges
    horizon = np.array(horizon)           #put all range values in an array
    if horizon is not None:           
        horizon[horizon >= 3.0] = 0

    sinx = np.sin(degree)                #getting degrees of each beam
    # print("sinx",sinx)
    cosx = np.cos(degree)
    # print("cosx",cosx)

    # print(horizon)

    y = horizon * sinx                    #multiply laser ranges with the sin and cos to get the radians 
    # print("Printing X",x)
    x = horizon * cosx
    # print("Printing Y",y)
    reshape_x = x.reshape((x.shape[0], 1))
    reshape_y = y.reshape((y.shape[0], 1))

    coord_X = []                     #To store the final X points
    coord_Y = []                     #To store the final Y points

    zeros_list[:, 0:1] = reshape_x
    zeros_list[:, 1:] = reshape_y
    
    x_coord = zeros_list[:,0]           #ranges X values           
    y_coord = zeros_list[:,1]           #ranges Y values

    lists = range(len(x_coord))        #Length of ranges i.e 361 values
    # print("printing list length= ",len(lists))
    
    scan_time = 0
    scanning = True
    k = 10
    
    
    for ran in range(5):
        inl = []
        outl = []

        for iter in range(k):
            inliers = []
            outliers = []

            beam1 = random.randint(0, 360)
            beam2 = random.randint(0, 360)
            # print("beam1",beam1)
            # print("beam2", beam2)
            random_point1 = lists[beam1]
            # print("random1", random_point1)
            random_point2 = lists[beam2]
            a2 = x_coord[random_point1]
            b2 = y_coord[random_point1]
            a3 = x_coord[random_point2]
            b3 = y_coord[random_point2]
            # print("a2=", a2)
            # print("a3=", a3)
            # print("b2= ", b2)
            # print("b3=", b3)
            # calculate(a2, b2, a3, b3,)
            for index in (lists):
                a1 = x_coord[index]
                b1 = y_coord[index]
                # print("A1", a1)
                numo = abs((a3- a1)*(b1-b2)-(a1-a2)*(b3-b1))
                deno = abs(math.sqrt(pow(a3-a1, 2)+pow(b3-b1, 2)))
                distance = abs(numo/deno)
                # print("Distance",distance)
                if distance < thresh:
                    inliers.append(index)
                else:
                    outliers.append(index)
                efficiency = (len(inliers)/(len(inliers) + len(outliers)))
                # print(efficiency)
                # print("Inliers", inliers)
                # print("Outliers", outliers)
            if len(inl) < len(inliers):
                inl = inliers
                outl = outliers
                # yout = zeros_list[inl,:]
                p1x = max(zeros_list[inl, 0])
                # print(p1x)
                p1y = min(zeros_list[inl, 0])
                p2x = max(zeros_list[inl, 1])
                p2y = min(zeros_list[inl, 1])
                    # print ("Inliers Count")
                # print (len(inl))
    coord_X.append(p1x)
    # print ("after adding")
    coord_Y.append(p2x)
    coord_X.append(p1y)
    coord_Y.append(p2y)
    # print("p1x=",p1x)
    # print("p2x=",p2x)
    # print("p1y=",p1y)
    # print("p2y=",p2y)
    lists = outl

    for i in range(2):
        if len(coord_X) > 1:
            p = Point()
            p.x = coord_X[i]
            p.y = coord_Y[i]
            marker.points.append(p)

    marker.scale.x = 0.05
    marker.scale.y = 0.0
    marker.scale.z = 0.0 
        
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.2
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    pub.publish(marker)
    # print("ending visualizer")



def get_streaks():
    global run_rate
    rospy.init_node("perception", anonymous=True)
    rospy.Subscriber("/base_scan", LaserScan, get_Ransac)
    rospy.spin()
    


if __name__ == "__main__":
    try :
        get_streaks()
    except rospy.ROSInterruptException:
        pass
