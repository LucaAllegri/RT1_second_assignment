# ASSIGNMENT 2 - RT1
This project implements a ROS 2 control system for a mobile robot in a 3D simulation environment (Gazebo/RViz).
The system allows manual teleoperation with an integrated safety layer that prevents collisions by monitoring laser scan data
and automatically reversing the robot when it gets too close to obstacles.

<div align="center">
  <img src="/image_forReadMe/MogiRobot.gif" alt="Simulazione Mogi Robot" width="400"/>
</div>
<br>

<br>🔴**CUSTOM COMMUNICATION INTERFACES**
<br>The project relies on a dedicated package, `robot_custom_msgs`, to define the specific data structures (messages and services) used by the nodes.

**CUSTOM MESSAGE:** 
- `ObstacleInfo.msg` (published by the Distance Node):
  - the distance to the nearest detected object
  - the sector where the nearest obstacle is located
  - the current safety threshold

**CUSTOM SERVICES:** 
- `AverageVelocity.srv`:
  - Request: Empty (---).
  - Response: Returns avg_linear_x and avg_angular_z.
- `Threshold.srv`:
  - Request: the new desired safety distance.
  - Response: confirmation of the updated threshold.

🔴**ARCHITECTURE AND NODES**
<br>The simulation is managed by a total of 4 nodes, launched simultaneously via a single launch file (`controller.launch.py`) located in the `/launch` directory.

**1) Simulation Environment $\rightarrow$** The launch file execute simultaneously the package of the simulation environment (`bme_gazebo_sensors`) which is responsible for:
 - running gazebo server/client
 - providing the URDF robot description and sensor
 - publishing sensor data

**2) UI NODE (InputController) $\rightarrow$**: it's the node that manages user input and is responsible for communicating motion commands. This node:
- provides in its terminal a keyboard interface for setting linear and angular velocities and changing the threshold;
- communicates with the Distance Node the velocity command for 3 second over an intermediate topic (not directly to `/cmd_vel`). The node then automatically stops the movement by publishing a zero velocity on the same `/intermediate_vel` topic;
- allows the user to dynamically change the safety threshold using the ROS service `Threshold.srv`;
- records velocity history to provide data for the `AverageVelocity.srv`.

<div align="center">
  <img src="/image_forReadMe/ui.png" alt="User Interface" width="700"/>
</div>
<br>

**3) DISTANCE NODE (DistanceController) $\rightarrow$**: it's the node that implements the safety logic of the system:
- it continuously checks the position of the mobile robot on `/goal_pose` topic;
- monitors the `/scan` topic to detect obstacles in 4 sectors, specifically monitoring if the robot is too close to an obstacle (the closeness depends on the actual threshold);
- if a "danger" position is detected:
  - communicates to the ui_node to stop requesting user input,
  - reverses the robot's direction of movement until it exits the forbidden area,
  - stops the robot,
  - updates the `/is_reversing` topic, allowing the ui_node to resume requesting user input;
- publishes the `ObstacleInfo.msg` to keep other nodes informed about the environment.
The distance_node's terminal acts as a log, printing distance, angle and direction of the relative obstacle.

<div align="center">
  <img src="/image_forReadMe/log.png" alt="log" width="700"/>
</div>
<br>

**4) STATUS NODE (StatusNode) $\rightarrow$**: The status node:
- subscribes to `/obstacle_info` topic;
- requests the new average velocity from the service;
- prints a summary whenever the robot stops, including:
  - minimum distance to obstacles,
  - obstacle sector,
  - actual threshold,
  - average linear and angular velocity.
<div align="center">
  <img src="/image_forReadMe/robot_status.png" alt="Robot Status" width="700"/>
</div>
<br>



<br>🔴**HOW TO RUN THE SIMULATION**
<br>To launch the entire simulation, just use the `controller.launch.py` file. 
<br>First, to build the workspace:
```bash
colcon build
source install/setup.bash
```

In a terminal launch the nodes using the command:
```bash
ros2 launch robot_controller controller.launch.py
```
This command will open the RViz and Gazebo environments, and three separate windows using xterm for the user inferface, distance log, and to see the robot's status.

Since the launch file uses the xterm -e prefix to open nodes in separate windows, make sure xterm is installed:
<br>If the launch fails, execute the following commands
```bash 
sudo apt update
sudo apt install xterm
```

In case the launch file does not work as expected, nodes can be started manually in the following order:
1) ```bash
   ros2 launch bme_gazebo_sensors spawn_robot.launch.py
2) ```bash
   ros2 run robot_controller ui
3) ```bash
   ros2 run robot_controller distance
4) ```bash
   ros2 run robot_controller status
 
