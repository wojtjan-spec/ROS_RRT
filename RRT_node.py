#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, String
from rrt import find_path_RRT, map_img

current_map = []
map_resolution = None
x_origin = None
y_origin = None
x_start_real = None
y_start_real = None
x_goal_real = None
y_goal_real = None
flag = True

def map_callback(msg):
	global current_map, map_resolution, x_origin, y_origin

	for i in range(msg.info.height):
		row = msg.data[i*msg.info.width:(i+1)*msg.info.width]
		current_map.append(row)
	
	map_resolution = msg.info.resolution	
	x_origin = msg.info.origin.position.x
	y_origin = msg.info.origin.position.y

	print("Received the map file. Transformed the map to 2D array.")	

def point_callback(msg):
	global x_start_real, y_start_real, x_goal_real, y_goal_real, flag
	x_start_real = msg.data[0]
	y_start_real = msg.data[1]
	x_goal_real = msg.data[2]
	y_goal_real = msg.data[3]
	flag = True
	
def get_index_from_coordinates(x_point, y_point):
	global map_resolution, x_origin, y_origin
	x_index = x_origin + int(round(x_point/map_resolution))
	y_index = y_origin + int(round(y_point/map_resolution))
	return x_index, y_index
	
def main():
	global flag
	rospy.init_node('RRT_node')
	rate = rospy.Rate(10)
	trajectory_publisher = rospy.Publisher('/trajectory', String, queue_size=10)
	rospy.Subscriber('/map', OccupancyGrid, map_callback)
	rospy.Subscriber('/start_goal', Float64MultiArray, point_callback)	
	
	while not rospy.is_shutdown():	
		rate.sleep()
		if(flag == True):
			x_start_index, y_start_index = get_index_from_coordinates(x_start_real, y_start_real)
			x_goal_index, y_goal_index = get_index_from_coordinates(x_goal_real, y_goal_real)

			start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
			print("Received new start and goal coordinates:")
			print("-------------------------------------------------")
			print("Start: ", start)
			print("Goal: ", goal)

			path, graph = find_path_RRT(start, goal, cv2.cvtColor(map_img(np.array(current_map)), cv2.COLOR_GRAY2BGR)[::-1])
			flag = False
			
			trajectory_publisher.publish(str(path))
			print("Sent calculated trajectory to motion_planner_node")
			print("-------------------------------------------------")
			rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

