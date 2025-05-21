#!/usr/bin/python
# Ntando Raji - 2584925, Jaedon Moodley - 2544456, Taruna Naidoo - 2546838, Yassir Ali - 2623035


"""
To input the position you'd like to move the drone, specify the x & y values as follows:
    ...navigator.py <x-coord>,<y-coord> so if I wanted to move the drone and have it land at position (4, -5). 
I would type: 
    ...navigator.py 4,-5.

"""

import rospy
import numpy as np
import sys
import tf
import re
import matplotlib.pyplot as plt

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.srv import GetModelState


Y_OFFSET = 257
X_OFFSET = 257


class PIDController:
    def __init__(self, goal):
        self.error_history = [np.zeros(shape=goal.shape)]
        self.goal = goal
    
    def compute(self, state, Kp, Ki, Kd):
        error = self.goal - state

        U = Kp * error + Ki * np.sum(np.array(self.error_history)) + Kd * (error - self.error_history[-1])

        self.error_history.append(error)

        return U


def get_model_info(model_name, relative_entity_name):
    """Gets the model's position and orientation"""
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        response = get_model_state(model_name=model_name, relative_entity_name=relative_entity_name)

        if not response.success:
            rospy.logwarn("Failed to get model state for: %s", model_name)
            return None, None

        model_position = np.array([response.pose.position.x, response.pose.position.y]) 
        model_orientation = np.array([response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z, response.pose.orientation.w])

        return model_position, model_orientation
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, None


def transform_velocity_to_local(vx, vy, yaw):
    """Transforms velocity from world frame to drone's local frame"""
    rotation_matrix = np.array([
        [np.cos(yaw), np.sin(yaw)],
        [-np.sin(yaw), np.cos(yaw)]
    ])
    
    local_velocity = np.matmul(rotation_matrix, np.array([vx, vy]))
    return local_velocity[0], local_velocity[1]


def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    
    return np.frombuffer(buffer,
            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
            count=int(width)*int(height),
            offset=len(header)
            ).reshape((int(height), int(width)))


def get_world_map():
    my_map = read_pgm("./src/survelliance_bot/src/maps/my_map.pgm").copy()
    return my_map[204:549, 305:712]


def talker():
    rospy.init_node('talker', anonymous=True)
    rospy.wait_for_service('/gazebo/get_model_state')

    rate = rospy.Rate(10) # 10hz
    rospy.sleep(1)

    goal_coords = np.array([int(i) for i in sys.argv[-1].split(",")])
    print(goal_coords)

    velocity = Twist()

    world_map = get_world_map()
    print(world_map)

    try:
        world_position, _ = get_model_info(model_name="mobile_base", relative_entity_name="world")
        model_position = np.array([Y_OFFSET - world_position[0], X_OFFSET + world_position[1]])
        print(model_position)

        # TODO: Add code to find a path

        # TODO: Navigate the turtlebot

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
