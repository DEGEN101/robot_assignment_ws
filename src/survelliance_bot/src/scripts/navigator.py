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
import random
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
        def is_valid(point, world_map):
            y, x = point
            h, w = world_map.shape
            return 0 <= y < h and 0 <= x < w and world_map[int(y), int(x)] == 255

        def euclidean(p1, p2):
            return np.linalg.norm(np.array(p1) - np.array(p2))

        def get_nearest(tree, point):
            return min(tree, key=lambda node: euclidean(node[0], point))

        def steer(from_point, to_point, step_size=10):
            direction = np.array(to_point) - np.array(from_point)
            length = np.linalg.norm(direction)
            if length == 0:
                return tuple(from_point)
            direction = (direction / length) * min(step_size, length)
            new_point = np.array(from_point) + direction
            return tuple(map(int, new_point))

        def is_collision_free(p1, p2, world_map):
            line = np.linspace(p1, p2, num=int(euclidean(p1, p2)))
            for point in line:
                if not is_valid(point, world_map):
                    return False
            return True

        # Prepare start and goal
        start = tuple(map(int, model_position))
        goal = tuple(map(int, [Y_OFFSET - goal_coords[0], X_OFFSET + goal_coords[1]]))

        # RRT implementation
        tree = [(start, None)]
        path = None
        max_iter = 1000
        goal_threshold = 10

        for _ in range(max_iter):
            if np.random.rand() < 0.1:
                sample = goal
            else:
                sample = (random.randint(0, world_map.shape[0] - 1),
                        random.randint(0, world_map.shape[1] - 1))

            if not is_valid(sample, world_map):
                continue

            nearest_node, _ = get_nearest(tree, sample)
            new_point = steer(nearest_node, sample)

            if is_valid(new_point, world_map) and is_collision_free(nearest_node, new_point, world_map):
                tree.append((new_point, nearest_node))

                if euclidean(new_point, goal) < goal_threshold:
                    print("Path found!")
                    path = [goal]
                    current = new_point
                    while current is not None:
                        path.append(current)
                        for node in tree:
                            if node[0] == current:
                                current = node[1]
                                break
                    path.reverse()
                    break

        if path is None:
            print("No path found")
        else:
            print("Planned path:", path)
            # Optional: visualize the path
            plt.imshow(world_map, cmap='gray')
            path_arr = np.array(path)
            plt.plot(path_arr[:, 1], path_arr[:, 0], color='red')
            plt.scatter(start[1], start[0], color='green')
            plt.scatter(goal[1], goal[0], color='blue')
            plt.title("RRT Path")
            plt.show()


        # TODO: Navigate the turtlebot
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pid = PIDController(np.array([0.0, 0.0]))

        Kp = 0.5
        Ki = 0.0
        Kd = 0.1

        print("Starting navigation...")

        for target_pixel in path[1:]:
            # Convert map pixel to world coordinate
            target_world = np.array([
                -(target_pixel[0] - Y_OFFSET),
                target_pixel[1] - X_OFFSET
            ])
            pid.goal = target_world

            while not rospy.is_shutdown():
                current_position, orientation = get_model_info("mobile_base", "world")
                if current_position is None:
                    continue

                error = pid.goal - current_position
                dist_to_goal = np.linalg.norm(error)

                if dist_to_goal < 0.2:
                    break  # Close enough to the waypoint

                velocity_cmd = pid.compute(np.array(current_position), Kp, Ki, Kd)

                # Optional: adjust orientation with yaw, currently we don't
                vx, vy = velocity_cmd
                velocity.linear = Vector3(vx, vy, 0)
                velocity.angular = Vector3(0, 0, 0)

                pub.publish(velocity)
                rate.sleep()

        # Stop the robot after reaching the goal
        velocity.linear = Vector3(0, 0, 0)
        velocity.angular = Vector3(0, 0, 0)
        pub.publish(velocity)
        print("Reached goal.")


    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass