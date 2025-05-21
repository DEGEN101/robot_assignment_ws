# Survelliance Robot Navigation Assignment

This guide explains how to set up and run the TurtleBot Gazebo simulation for robot navigation and SLAM using Singularity.

---

## 🚀 Launch Singularity and Set Up ROS

```bash
cd ~/ros
singularity run --bind /mnt/wslg ./wits-ros.simg
```

Once inside the Singularity terminal:

```bash
git clone https://github.com/DEGEN101/robot_assignment_ws.git
cd robot_assignment_ws
catkin_make
```

---

## ❗ Troubleshooting: `catkin_make` CMake Error

If you see the following error:

\`\`\`
CMake Error: The source "/home/keren/ros_home/robot_assignment_ws/src/CMakeLists.txt" does not match the source ...
\`\`\`

Fix it by deleting the `build` and `devel` folders:

```bash
rm -rf build devel
catkin_make
```

---

## 🌍 Launch the Gazebo World

In **Terminal 1**:

```bash
cd ~/robot_assignment_ws
source devel/setup.bash
./startworld
# or alternatively
roslaunch turtlebot_gazebo turtlebot_world.launch
```

---

## 🗺️ Start SLAM (GMapping)

In **Terminal 2**:

```bash
roslaunch turtlebot_gazebo gmapping_demo.launch
```

Custom Config
```bash
roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=./src/survelliance_bot/launch/gmapping_config.launch
```

---

## ⌨️ Install Teleop (Run Once Only)

```bash
cd ~/robot_assignment_ws/src
git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
cd ~/robot_assignment_ws
catkin_make
source devel/setup.bash
```

---

## 🕹️ Drive the Robot to Build the Map

In **Terminal 3**:

```bash
python $(rospack find teleop_twist_keyboard)/teleop_twist_keyboard.py cmd_vel:=/cmd_vel_mux/input/teleop
```

### Keyboard Controls

- \`i\` = forward  
- \`,` = backward  
- \`j\` = turn left  
- \`l\` = turn right  
- \`k\` = stop  

> ⚠️ Keep the terminal in focus while driving!

Drive around the environment to help GMapping build a complete map.

---

## 🛰️ View Mapping Progress in RViz

In **Terminal 4**:

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```
---

### 💾 Save the Map

After mapping:

```bash
rosrun map_server map_saver -f ./src/survelliance_bot/src/maps/my_map
```
---

## 📌 3. Localization with AMCL

Launch AMCL using the saved map:

```bash
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/degen101/ros_home/robot_assignment_ws/src/survelliance_bot/src/maps/my_map.yaml
```

> ❗ Replace with the full path to your `.yaml` map file.

---

## 👁️ 4. Viewing in RViz

If RViz doesn't open automatically, run:

```bash
rviz
```

In RViz:

1. Set `Fixed Frame` to `map`
2. Add:
   - `Map` (topic: `/map`)
   - `RobotModel`
   - `LaserScan` (topic: `/scan`)
3. Use the `2D Nav Goal` tool to send a goal to the robot

---

## 🛠️ 5. Troubleshooting

### 🔧 `Fixed Frame [map] does not exist`
- Ensure AMCL is publishing the `map → odom` TF
- Make sure `/map` and `/tf` topics are active:
  ```bash
  rostopic echo /map
  rostopic echo /tf
  ```

### ❌ `map_server` crashed with exit code 255
- You probably passed a `.pgm` file instead of a `.yaml`
- Always use:
  ```bash
  map_file:=/full/path/to/my_map.yaml
  ```

---

## 🧭 Run the Navigation Node

In a **new terminal**:

```bash
rosrun surveillance_bot navigator.py
```

You will be prompted to enter \`x\` and \`y\` coordinates.  
The robot will drive to the point using \`/gazebo/get_model_state\` and velocity control.

---
