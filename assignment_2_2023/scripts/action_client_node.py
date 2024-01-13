#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningFeedback, PlanningResult
from assignment_2_2023.msg import CustomMessage
from nav_msgs.msg import Odometry

def set_goal_from_user_input():
    while not rospy.is_shutdown():
        try:
            # Ask the user whether to enter a target or cancel
            choice = input("Enter 't' for a new target or 'c' to cancel: ")

            if choice.lower() == 't':
                # Get target position from user input
                x = float(input("Enter target x position: "))
                y = float(input("Enter target y position: "))
                # Call the set_goal function with user-input target position
                set_goal(x, y)
            elif choice.lower() == 'c':
                # Call the cancel_goal function
                cancel_goal()
            else:
                rospy.logwarn("Invalid choice. Enter 't' for a new target or 'c' to cancel.")
        except ValueError:
            rospy.logerr("Invalid input. Please enter numerical values.")
            continue

def set_goal(x, y):
    # Create an action client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()

    # Create a goal with the specified target position (x, y)
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Send the goal to the action server
    client.send_goal(goal, feedback_cb=feedback_callback)

    # Wait for the action to finish
    client.wait_for_result()

    # Check if the action was successful
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Target reached successfully!")
    else:
        rospy.logwarn("Failed to reach the target.")

def cancel_goal():
    # Create an action client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()

    # Cancel the goal
    client.cancel_goal()

def feedback_callback(feedback):
    # Feedback callback to monitor the progress
    rospy.loginfo("Current pose: {}".format(feedback.actual_pose))
    # You can add additional processing based on feedback if needed
    
    
def odom_callback(odom_msg):
    # Process the Odometry message and extract relevant information
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    vel_x = odom_msg.twist.twist.linear.x
    vel_z = odom_msg.twist.twist.angular.z

    # Create a CustomMessage with the extracted information
    custom_msg = CustomMessage()
    custom_msg.x = x
    custom_msg.y = y
    custom_msg.vel_x = vel_x
    custom_msg.vel_z = vel_z

    # Publish the CustomMessage on the /custom_topic (or /odom) topic
    custom_pub.publish(custom_msg)

if __name__ == "__main__":
    rospy.init_node('action_client_node')

    # Subscriber to the /odom topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher for the custom message on the /custom_topic topic
    custom_pub = rospy.Publisher('/custom_topic', CustomMessage, queue_size=10)

    set_goal_from_user_input()

    # cancel_goal()

    rospy.spin()

