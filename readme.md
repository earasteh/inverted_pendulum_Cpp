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

## Configuration

The configuration of the pendulum simulator and controller is done through a YAML file. This provides an easy way to tune parameters without changing the source code.

### Pendulum Simulator Configuration

The pendulum simulator's configuration includes parameters such as mass (`m`), gravity (`g`), length (`L`), integrator type (`integrator`), and simulation mode (`sim_mode`).

- `m`: The mass of the pendulum.
- `g`: The acceleration due to gravity.
- `L`: The length of the pendulum.
- `integrator`: The type of numerical integration to use for the simulation. Choices are "Euler" for Euler's method, or "RK4" for the 4th order Runge-Kutta method.
- `sim_mode`: The simulation mode. "free-fall" for uncontrolled simulation where the pendulum is free to swing, "controlled" where the pendulum controller is active and "perturb" where there is active control but perturbation is added to torque input.

Example configuration:

```yaml
pendulum_simulator:
    ros__parameters:
        m: 1.0
        g: 9.81
        L: 1.0
        integrator: "RK4"
        sim_mode: "controlled"
```

### Pendulum Controller Configuration

The pendulum controller's configuration includes parameters such as `K_theta` and `K_omega`, which are the gain values for the LQR controller.

- `K_theta`: The gain for the pendulum angle error.
- `K_omega`: The gain for the angular velocity error.

Example configuration:

```yaml
pendulum_controller:
    ros__parameters:
        K_theta: 30.0
        K_omega: 30.0
```

These parameters are loaded at startup from the specified YAML file using the `--params-file` argument. They can be adjusted as needed to change the behavior of the pendulum simulator and controller.

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

