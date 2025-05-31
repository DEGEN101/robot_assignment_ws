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


# Coordinates (center of full map near robot starting point)
FULL_CENTER_Y = 460  # 204 + 256
FULL_CENTER_X = 561  # 305 + 256

# Crop bounds
CROP_Y_START = 204
CROP_X_START = 305

# Corrected offset for cropped map
Y_OFFSET = FULL_CENTER_Y - CROP_Y_START  # 460 - 204 = 256
X_OFFSET = FULL_CENTER_X - CROP_X_START  # 561 - 305 = 256




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

        # Just extract raw world coordinates (x, y)
        model_position = np.array([response.pose.position.x, response.pose.position.y]) 
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

# def get_world_map():
#     return read_pgm("./src/survelliance_bot/src/maps/my_map.pgm").copy()


def find_nearest_valid(point, world_map, max_radius=20):
    y, x = point
    h, w = world_map.shape

    for radius in range(1, max_radius + 1):
        for dy in np.random.permutation(range(-radius, radius + 1)):
            for dx in np.random.permutation(range(-radius, radius + 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w:
                    if world_map[int(ny), int(nx)] >= 250:  # relaxed threshold
                        return (ny, nx)
    return None


def talker():
    import random 
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
        model_position_world, _ = get_model_info("mobile_base", "world")
        model_position = np.array([
            Y_OFFSET - model_position_world[0],
            X_OFFSET + model_position_world[1]
        ])


        print(model_position)

 # TODO: Add code to find a path
        def is_valid(point, world_map):
            y, x = point
            h, w = world_map.shape
            if not (0 <= y < h and 0 <= x < w):
                return False
            return int(world_map[int(y), int(x)]) >= 250  # Accept near-white pixels too


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

                # Check surrounding pixels in a small radius (e.g., 2 pixels)
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < world_map.shape[0] and 0 <= nx < world_map.shape[1]:
                            if world_map[ny, nx] < 250:
                                return False
            return True





        # Prepare start and goal
        raw_start = tuple(map(int, model_position))
        raw_goal = tuple(map(int, [Y_OFFSET - goal_coords[0], X_OFFSET + goal_coords[1]]))

        start = find_nearest_valid(raw_start, world_map)
        goal = find_nearest_valid(raw_goal, world_map)

        print("Raw start:", raw_start)
        print("Adjusted start:", start)
        print("Raw goal:", raw_goal)
        print("Adjusted goal:", goal)

        # # Visualization to debug positioning
        # plt.imshow(world_map, cmap="gray")
        # plt.title("Map with Start/Goal (raw positions)")

        # # Overlay raw positions
        # ry, rx = raw_start
        # gy, gx = raw_goal
        # plt.scatter(rx - 305, ry - 204, color='green', label='Start')  # adjusted for cropping
        # plt.scatter(gx - 305, gy - 204, color='blue', label='Goal')

        # plt.legend()
        # plt.show()


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
            return  # <- EXIT before trying to navigate
        else:
            print("Planned path:", path)
            plt.imshow(world_map, cmap='gray')
            path_arr = np.array(path)
            plt.plot(path_arr[:, 1], path_arr[:, 0], color='red', linewidth=2)
            plt.scatter(start[1], start[0], color='green', label='Start')
            plt.scatter(goal[1], goal[0], color='blue', label='Goal')
            plt.title("Planned RRT Path")
            plt.legend()

            # Save the plot to file instead of showing
            filename = "path_to_{}_{}.png".format(goal_coords[0], goal_coords[1])
            plt.savefig(filename)
            plt.close()
            print("Saved path plot as {}".format(filename))

        


        # TODO: Navigate the turtlebot
        pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        # pid = PIDController(np.array([0.0, 0.0]))

        # Kp = 0.5
        # Ki = 0.0
        # Kd = 0.1

        print("Starting navigation...")

        def find_lookahead_point(current_pos, path, lookahead_dist=0.5):
            for pt in path:
                if np.linalg.norm(np.array(pt) - current_pos) > lookahead_dist:
                    return np.array([
                        -(pt[0] - Y_OFFSET),
                        pt[1] - X_OFFSET
                    ])
            return np.array([
                -(path[-1][0] - Y_OFFSET),
                path[-1][1] - X_OFFSET
            ])  # fallback to goal


        # ---------------------------
        # PURE PURSUIT NAVIGATION
        # ---------------------------

        lookahead_dist = 0.5
        print("Starting navigation...")

        while not rospy.is_shutdown():
            current_position, orientation = get_model_info("mobile_base", "world")
            if current_position is None:
                continue

            # Get next target to follow
            target = find_lookahead_point(current_position, path, lookahead_dist)

            # Distance to goal
            dist_to_goal = np.linalg.norm(np.array(target) - np.array(current_position))
            if dist_to_goal < 0.3:
                break  # close enough to stop

            # Heading angle
            target_angle = np.arctan2(target[1] - current_position[1], target[0] - current_position[0])
            _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
            angle_diff = (target_angle - yaw + np.pi) % (2 * np.pi) - np.pi

            # Control law
            linear_speed = 0.3
            angular_speed = 1.0 * angle_diff

            velocity.linear = Vector3(linear_speed, 0, 0)
            velocity.angular = Vector3(0, 0, angular_speed)

            pub.publish(velocity)
            print("Current pos:", current_position)
            print("Lookahead target:", target)
            print("Angle diff:", angle_diff)

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
