# Pendulum Control with ROS2

This project implements a pendulum control system using ROS2. The system simulates a pendulum and applies control techniques to maintain the pendulum at an upright position. 

## Setup and Installation

1. Install ROS2 Humble by following the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. Create a new ROS2 workspace or navigate to your existing workspace:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
3. Clone the pendulum control project into your workspace's `src` directory.
4. Build the project using colcon:
```
cd ~/ros2_ws
colcon build --packages-select pendulum_control
```
5. Source your ROS2 workspace:
```
source ~/ros2_ws/install/setup.bash
```
## Running the Project

First, start the pendulum controller:
```
ros2 run pendulum_control pendulum_controller --ros-args --params-file pendulum_control/config/pendulum_parameters.yaml
```
Then, in another terminal, start RViz2:
```
ros2 run rviz2 rviz2
```
Finally, in a third terminal, start the pendulum simulator:
```
ros2 run pendulum_control pendulum_simulator --ros-args --params-file pendulum_control/config/pendulum_parameters.yaml
```

## Project Structure

The project structure is as follows:

- `config`: This folder contains the YAML file for setting parameters like mass, gravity, length, integrator type, and control mode for the pendulum simulator, and the gain values for the controller.
- `src`: This folder contains the source files for the pendulum simulator and controller nodes.

## Pendulum Equations of Motion

The motion of the pendulum is governed by the following second-order differential equation, derived from Newton's second law:

theta'' = - (g/L) * sin(theta)

where:
- theta is the pendulum angle,
- g is the acceleration due to gravity,
- L is the length of the pendulum.

## Controls

The control system uses a Linear Quadratic Regulator (LQR) to keep the pendulum at an upright position. The controller calculates the torque needed to maintain this position based on the current angle and angular velocity of the pendulum.

The LQR controller requires gain values (K_theta for angle and K_omega for angular velocity). These gains are chosen based on the dynamics of the pendulum system and the desired response characteristics. They can be tuned to make the controller more or less aggressive.

