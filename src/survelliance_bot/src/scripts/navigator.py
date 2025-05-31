#!/usr/bin/python
# Ntando Raji - 2584925, Jaedon Moodley - 2544456, Taruna Naidoo - 2546838, Yassir Ali - 2623035


"""
To input the position you'd like to move the drone, specify the x & y values as follows:
    ...navigator.py <x-coord>,<y-coord> so if I wanted to move the drone and have it land at position (4, 6). 
I would type: 
    ...navigator.py 4,6.

"""

import rospy
import numpy as np
import sys
import tf
import re
import matplotlib.pyplot as plt
import random

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.srv import GetModelState


# Coordinates (center of full map near robot starting point)
FULL_CENTER_Y = 464
FULL_CENTER_X = 564

# Crop bounds
CROP_Y_START = 204
CROP_X_START = 305

# Corrected offset for cropped map
Y_OFFSET = FULL_CENTER_Y - CROP_Y_START  # 460 - 204 = 256
X_OFFSET = FULL_CENTER_X - CROP_X_START  # 561 - 305 = 256

# World-to-map scale based on wall measurements
SCALE_X_M_PER_PX = 0.05
SCALE_Y_M_PER_PX = 0.049


class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=[np.inf, np.inf]):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits

        self.integral = None
        self.prev_error = None
    
    def compute(self, error, dt):
        
        if self.integral is None:
            self.integral = np.zeros(error.shape)

        if self.prev_error is None:
            self.prev_error = np.zeros(error.shape)

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        U = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error

        return np.maximum(self.output_limits[0], np.minimum(U, self.output_limits[1]))


def get_model_info(model_name, relative_entity_name):
    """Gets the model's position and orientation"""
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        response = get_model_state(model_name=model_name, relative_entity_name=relative_entity_name)

        if not response.success:
            rospy.logwarn("Failed to get model state for: %s", model_name)
            return None, None

        # Just extract raw world coordinates (y, x)
        model_position = np.array([response.pose.position.y, response.pose.position.x]) 
        model_orientation = np.array([
            response.pose.orientation.x,
            response.pose.orientation.y,
            response.pose.orientation.z,
            response.pose.orientation.w
        ])

        return model_position, model_orientation

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None, None


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


def is_valid(point, world_map, radius=6):
    y, x = point

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            ny, nx = y + dy, x + dx
            if 0 <= ny < world_map.shape[0] and 0 <= nx < world_map.shape[1]:
                if world_map[ny, nx] < 250:
                    return False
    return True


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
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)

    dist = np.linalg.norm(p2 - p1)
    if dist < 1.0:
        return True

    num_points = max(int(dist), 2)
    y_vals = np.linspace(p1[0], p2[0], num=num_points)
    x_vals = np.linspace(p1[1], p2[1], num=num_points)

    for y, x in zip(y_vals, x_vals):
        y, x = int(y), int(x)

        if not is_valid([y, x], world_map, radius=4):
            return False

    return True


def talker():
    rospy.init_node('navigator', anonymous=True)
    rospy.wait_for_service('/gazebo/get_model_state')

    rate = rospy.Rate(10) # 10hz
    rospy.sleep(1)

    goal_coords = np.array([int(i) for i in sys.argv[-1].split(",")][::-1])
    goal_position = [
        int(Y_OFFSET - (goal_coords[0] / SCALE_Y_M_PER_PX)),  # convert meters to pixels
        int(X_OFFSET + (goal_coords[1] / SCALE_X_M_PER_PX))
    ]
    velocity = Twist()

    world_map = get_world_map()

    try:
        model_position_world, _ = get_model_info("mobile_base", "world")
        model_position = np.array([
            int(Y_OFFSET - (model_position_world[0] / SCALE_Y_M_PER_PX)),
            int(X_OFFSET + (model_position_world[1] / SCALE_X_M_PER_PX))
        ], dtype=np.int32)


        #TODO: Path Planning
        start = tuple(model_position)
        goal = tuple(goal_position)


        if start is None or goal is None:
            print("Could not find valid start or goal nearby.")
            return

        print("Start:", start)
        print("Goal:", goal)
        print("Valid start:", is_valid(start, world_map))
        print("Valid goal:", is_valid(goal, world_map))


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
            new_point = steer(nearest_node, sample, step_size=20)

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
            return  # <- EXIT before trying to navigate
        else:
            print("Planned path:", path)
            plt.imshow(world_map, cmap='gray')
            path_arr = np.array(path)
            plt.scatter(path_arr[:, 1], path_arr[:, 0], color='red', linewidth=2, s=20)
            plt.scatter(start[1], start[0], color='green', label='Start', s=20)
            plt.scatter(goal[1], goal[0], color='blue', label='Goal', s=20)
            plt.title("Planned RRT Path")
            plt.legend()

            # Save the plot to file instead of showing
            filename = "path_to_{}_{}.png".format(goal_coords[0], goal_coords[1])
            plt.savefig(filename)
            plt.show()
            plt.close()
            print("Saved path plot as {}".format(filename))


        # TODO: Navigate the turtlebot
        pub_move = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # ---------------------------
        # PID Control Navigation
        # ---------------------------

        print("Starting navigation...")

        last_time = rospy.Time.now()

        for target_position in path[1:]:
            model_position_world, _ = get_model_info(model_name="mobile_base", relative_entity_name="world")
            model_position = np.array([
                int(Y_OFFSET - (model_position_world[0] / SCALE_Y_M_PER_PX)),
                int(X_OFFSET + (model_position_world[1] / SCALE_X_M_PER_PX))
            ], dtype=np.int32)
            
            target_position_world = [
                (Y_OFFSET - target_position[0]) * SCALE_Y_M_PER_PX, 
                (target_position[1] - X_OFFSET) * SCALE_X_M_PER_PX  
            ]

            dx = target_position_world[1] - model_position_world[1]
            dy = target_position_world[0] - model_position_world[0]
            target_yaw = np.arctan2(dy, dx)

            # PID controllers for movement & orientation
            movement_controller = PIDController(Kp=0.4, Ki=0.001, Kd=0.1, output_limits=(-0.5, 0.5))
            orientation_controller = PIDController(Kp=0.6, Ki=0.001, Kd=0.1, output_limits=(-1.5, 1.5))
            
            orientation_corrected = False

            while not rospy.is_shutdown():
                model_position_world, model_orientation = get_model_info(model_name="mobile_base", relative_entity_name="world")
                model_position = np.array([
                    int(Y_OFFSET - (model_position_world[0] / SCALE_Y_M_PER_PX)),
                    int(X_OFFSET + (model_position_world[1] / SCALE_X_M_PER_PX))
                ], dtype=np.int32)

                _, _, model_yaw = tf.transformations.euler_from_quaternion(model_orientation)
                yaw_error = (target_yaw - model_yaw + np.pi) % (2 * np.pi) - np.pi
                distance_error = np.linalg.norm(target_position - model_position)
                
                now = rospy.Time.now()
                dt = (now - last_time).to_sec()
                last_time = now

                # Check if the turtlebot is facing the right direction - facing towards the target
                if abs(yaw_error) < 0.02:
                    velocity.angular.z = 0
                    orientation_corrected = True

                # Update the turtlebot's orientation
                if not orientation_corrected:
                    z_rotation = orientation_controller.compute(np.array([yaw_error]), dt)[0]
                    velocity.angular.z = z_rotation

                if orientation_corrected:
                    linear_vel = movement_controller.compute(np.array([distance_error]), dt)[0]

                    velocity.linear = Vector3(linear_vel, 0, 0)

                    # Check if the turtlebot is in the right location
                    if distance_error <= 2:
                        velocity.linear = Vector3(0, 0, 0)
                        pub_move.publish(velocity) 
                        break
                
                pub_move.publish(velocity)
                rate.sleep()

                print("------- Logging Model Info -------")
                print("Model position:", model_position)
                print("Target position:", target_position)
                print("Distance error", distance_error)
                print("Yaw error:", yaw_error)
                print("----------------------------------")


        # Stop the robot after reaching the goal
        velocity.linear = Vector3(0, 0, 0)
        velocity.angular = Vector3(0, 0, 0)
        pub_move.publish(velocity)
        print("Reached goal.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
