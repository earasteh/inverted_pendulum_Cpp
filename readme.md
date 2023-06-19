Pendulum Control System Using ROS2 and LQR
This project is a demonstration of controlling an inverted pendulum using ROS2 and a Linear Quadratic Regulator (LQR). The pendulum system is simulated using a physics model and the control strategy is implemented using ROS2 nodes.

Project Structure
The project is structured as follows:

config/: Contains YAML configuration files for the simulator and the controller.
src/: Contains the source code files for the simulator and the controller.
The primary components of the project are the pendulum_simulator and pendulum_controller nodes. The pendulum_simulator simulates the physics of an inverted pendulum while the pendulum_controller node applies a control input to stabilize the pendulum using an LQR controller.

Pendulum Dynamics
The dynamics of the pendulum are derived from the Euler-Lagrange equation, which results in a second-order differential equation. This equation is then linearized around the unstable equilibrium point (the inverted position), and discretized for implementation in the simulator node.

Control Strategy
The controller is based on a Linear Quadratic Regulator (LQR), which is a control strategy used to stabilize linear systems. Given a model of the system, an LQR controller optimally determines the control inputs to minimize a cost function, typically the sum of the state and control input energy. The resulting LQR gain was computed offline and is used in the pendulum_controller node.

Installation and Setup
The project is based on ROS2 Foxy. If you haven't already installed ROS2, follow the instructions on the official ROS2 documentation.

To compile the project:

Navigate to your ROS2 workspace's src directory.

Clone the repository:

bash
Copy code
git clone <repository_url>
Navigate back to your ROS2 workspace's root directory.

Compile the code:

bash
Copy code
colcon build
Source the workspace:

bash
Copy code
source install/setup.bash
Running the Project
Before running the project, make sure you're in the project's root directory and have sourced the workspace.

To run the project, use the following ROS2 commands:

Start the pendulum simulator:

bash
Copy code
ros2 run pendulum_control pendulum_simulator --ros-args --params-file pendulum_control/config/pendulum_parameters.yaml
Start RViz2:

bash
Copy code
rviz2
Load the provided configuration file in RViz2 to view the pendulum system. This will allow you to visualize the pendulum's motion in real-time.

Start the pendulum controller:

bash
Copy code
ros2 run pendulum_control pendulum_controller --ros-args --params-file pendulum_control/config/pendulum_parameters.yaml
This starts the controller which uses the LQR control strategy to regulate the pendulum's motion.
