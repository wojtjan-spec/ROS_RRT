#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, String

flag = True
def trajectory_callback(msg):
	global flag
	flag = True
	print("Received trajectory: ", msg.data)
	
	
def get_start_goal_points():
	global start_point, goal_point, flag
	print("Set new start and goal coordinates.")
	print("----------------------------------------------------")
	start_point = [float(input("Enter start point x coordinate: ")),
		       float(input("Enter start point y coordinate: "))]

	goal_point = [float(input("Enter goal point x coordinate: ")),
		      float(input("Enter goal point y coordinate: "))]
	flag = False
	
def main():
	global flag
	rospy.init_node('motion_planner_node')	
	start_goal_publisher = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)
	rospy.Subscriber('/trajectory', String, trajectory_callback)
	rate = rospy.Rate(10)
	message = None

	while not rospy.is_shutdown():
		if(flag == True):
			get_start_goal_points()
			message = Float64MultiArray(data=start_point+goal_point)
			flag = False
		start_goal_publisher.publish(message)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

