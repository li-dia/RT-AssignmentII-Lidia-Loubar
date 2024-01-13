#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Coordinates, CoordinatesResponse
from assignment_2_2023.msg import PlanningActionGoal

# Global variable to store the last target coordinates
last_target_coordinates = {'x': 0.0, 'y': 0.0}

def goal_callback(msg):
    # Extract target coordinates from the PlanningActionGoal message
    last_target_coordinates['x'] = msg.goal.target_pose.pose.position.x
    last_target_coordinates['y'] = msg.goal.target_pose.pose.position.y


def coordinates_service_callback(req):
    # Service callback function to handle requests
    response = CoordinatesResponse()
    response.x = last_target_coordinates['x']
    response.y = last_target_coordinates['y']
    return response

if __name__ == "__main__":
    rospy.init_node('last_target_service_node')

    # Subscribe to the /reaching_goal/goal topic
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, goal_callback)

    # Create a service server
    service = rospy.Service('/get_last_target_coordinates', Coordinates, coordinates_service_callback)

    rospy.spin()

