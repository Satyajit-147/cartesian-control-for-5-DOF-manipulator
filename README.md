# ROS 2 Cartesian Control via MoveIt Servo

## Overview
This repository contains the configuration and bridging scripts necessary to achieve real-time **Cartesian control** of a 5-DOF robotic manipulator using ROS 2 and MoveIt Servo.

### The Problem This Solves
Traditionally, robotic arms are controlled in **Joint Space**, meaning the operator has to manually calculate and move each individual motor (Joint 1, Joint 2, etc.) to get the tool to a specific location. For a 5-DOF manipulator, this is highly non-intuitive and difficult to coordinate for smooth, linear movements.

**Cartesian Control** solves this by letting you pilot the *end-effector* (the tip of the robot arm) directly in 3D space (X, Y, Z axes). You simply tell the robot "move forward" or "move up," and MoveIt Servo calculates the complex Inverse Kinematics (IK) in real-time, automatically adjusting all 5 joints simultaneously to achieve that smooth, straight-line motion.

---

## Execution Guide

To run the Cartesian control simulation/hardware, you will need to open **four separate terminals** and run the following commands in sequence.

### Step 1: Build and Source the Workspace
Before starting, ensure your underlying MoveIt 2 installation is sourced, then build and source your workspace.

**Terminal 1:**
bash
# Source the main ROS 2 and MoveIt 2 underlay (adjust path if your MoveIt2 source is different)
source /opt/ros/humble/setup.bash 

# Navigate to your workspace
cd ~/ws_manipulator4

# Build the workspace
colcon build

# Source the newly built workspace
source install/setup.bash


### Step 2: Launch MoveIt Servo
This brings up the simulation environment, the controllers, and the MoveIt Servo node which handles the real-time Cartesian calculations.

**Terminal 1 (Continued):**
bash
ros2 launch manipulator1_moveit_servo2 servo.launch.py

*(Wait for RViz to fully load and the terminal output to settle.)*

### Step 3: Activate the Servo Controller
By default, MoveIt Servo starts in a paused/sleeping state for safety. You must trigger a service call to wake it up and switch MoveIt from standard trajectory planning to real-time streaming control.

**Terminal 2:**
bash
source ~/ws_manipulator4/install/setup.bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

*(You should see a `success=True` response in the terminal.)*

### Step 4: Run the Keyboard Bridge
The standard ROS 2 keyboard controller outputs a `geometry_msgs/Twist` message. However, MoveIt Servo strictly requires a `geometry_msgs/TwistStamped` message. 

**What this bridge does:** It acts as a real-time translator. It listens to your keyboard inputs, attaches the current ROS clock timestamp, adds the `base_link` reference frame, and forwards the properly packaged data to the Servo node.

**Terminal 3:**
bash
source ~/ws_manipulator4/install/setup.bash
python3 keyboard_bridge.py


### Step 5: Run the Teleop Keyboard
Finally, launch the node that will capture your keystrokes. 

**Terminal 4:**
bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard


---

## Controls Mapping

Make sure **Terminal 4** is the active window (click on it). Use the following specific keys to pilot the manipulator's end-effector in Cartesian space:

| Key Command | Action (Cartesian Movement) |
| :--- | :--- |
| **`Shift` + `J`** | Move Forward (+X) |
| **`Shift` + `L`** | Move Backwards (-X) |
| **`t`** | Move Up (+Z) |
| **`b`** | Move Down (-Z) |
| **`i`** | Move Right (-Y) |
| **`,`** (comma) | Move Left (+Y) |
| **`k`** | **EMERGENCY STOP** (Halts all movement) |

*(Note: You can use `q` / `z` to increase or decrease the overall maximum speed within the teleop terminal).*
