# Ackermann Autonomous Car Simulation

## → Requirements

Before building and running the package, install the necessary dependencies using the `$ROS_DISTRO` environment variable:

```python
sudo apt-get install libeigen3-dev
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

### → Step 1: Create a ROS2 Workspace

1. Open a terminal and **Set up ROS2**.
    
    ```bash
    source /opt/ros/foxy/setup.bash #for ros2 foxy
    source /opt/ros/humble/setup.bash #for ro2 humble
    ```
    
2. Create a directory for the workspace and navigate into it:
    
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
    

### → Step 2: Clone the GitHub Repository

1. Navigate to the `src` directory:
    
    ```bash
    cd ~/ros2_ws/src
    ```
    
2. Clone the repository:
    
    ```
    git clone https://github.com/nailsnails/bearcat_simulation.git
    ```
    

### → Step 3: Build the Package

1. Go back to the root of the workspace:
    
    ```bash
    cd ~/ros2_ws
    ```
    
2. Build the package:
    
    ```bash
    colcon build --packages-select niagara_model
    colcon build --packages-select velodyne_gazebo_plugins
    ```
    
3. Source the workspace:
    
    ```bash
    source install/setup.bash
    ```
    

### → Step 4: Run the Simulation

1. To launch with Gazebo and Rviz:
    
    ```bash
    ros2 launch niagara_model display.launch.py
    ```
    
2. To launch with only Rviz:
    
    ```bash
    ros2 launch niagara_model display_gui.launch.py
    ```
    
### Step 5: Moving the Bearcar with the `teleop_twist_keboard`

When the simulation is running, start the `twist_to_ackermann` and the `teleop_twist_keboard` package each in a terminal. Steer the car with the commands of the `teleop_twist_keyboard`. The Bearcar should move now and you should see the visualization of lidar and camera data in RViz.  

### Alternative Step 5 : Moving the Car with `cmd_vel`

1. You can manually move the car using the `cmd_vel` command in the terminal. This command publishes velocity messages to control the car's movement:
    
    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
    ```
    
    - Replace the values in `{linear: {x: ...}}` and `{angular: {z: ...}}` to adjust the car's speed and direction:
        - `linear.x`: Controls forward and backward motion (positive values move forward, negative values move backward).
        - `angular.z`: Controls the turning rate (positive values turn left, negative values turn right).


   
