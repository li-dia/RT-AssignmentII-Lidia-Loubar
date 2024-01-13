#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import RobotInfo, RobotInfoResponse
from assignment_2_2023.msg import CustomMessage
from assignment_2_2023.srv import Coordinates 

# Global variables to store robot information
robot_position = {'x': 0.0, 'y': 0.0}
robot_speed_sum = 0.0
robot_speed_count = 0
last_target_coordinates = {'x': 0.0, 'y': 0.0}

def custom_callback(msg):
    # Process the CustomMessage and update robot position and speed
    global robot_position, robot_speed_sum, robot_speed_count

    robot_position['x'] = msg.x
    robot_position['y'] = msg.y

    robot_speed_sum += abs(msg.vel_x) + abs(msg.vel_z)
    robot_speed_count += 1

def get_last_target_coordinates():
    # Service client to retrieve the last target coordinates
    rospy.wait_for_service('/get_last_target_coordinates')
    try:
        client = rospy.ServiceProxy('/get_last_target_coordinates', Coordinates)
        response = client()
        return response.x, response.y
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None

def robot_info_service_callback(req):
    # Service callback function to handle requests
    global robot_position, robot_speed_sum, robot_speed_count, last_target_coordinates

    # Retrieve the last target coordinates
    last_target_coordinates['x'], last_target_coordinates['y'] = get_last_target_coordinates()

    # Calculate the distance to the target
    distance_to_target = ((robot_position['x'] - last_target_coordinates['x'])**2 +
                          (robot_position['y'] - last_target_coordinates['y'])**2)**0.5

    # Calculate the average speed
    average_speed = robot_speed_sum / robot_speed_count if robot_speed_count > 0 else 0.0

    # Reset the speed variables for the next calculation
    robot_speed_sum = 0.0
    robot_speed_count = 0

    response = RobotInfoResponse()
    response.distance_to_target = distance_to_target
    response.average_speed = average_speed
    return response

if __name__ == "__main__":
    rospy.init_node('robot_info_service_node')

    # Subscribe to the custom message topic
    rospy.Subscriber('/custom_topic', CustomMessage, custom_callback)

    # Create a service server
    service = rospy.Service('/get_robot_info', RobotInfo, robot_info_service_callback)

    rospy.spin()

