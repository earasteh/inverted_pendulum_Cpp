#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pendulum_control/msg/pendulum_state.hpp"  // Assume you have created this custom message
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PendulumSimulator : public rclcpp::Node
{
public:
    PendulumSimulator() : Node("pendulum_simulator"), current_angle_(0), current_angular_velocity_(0)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        control_input_publisher_ = this->create_publisher<pendulum_control::msg::PendulumState>("control_input", 10);
        pendulum_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pendulum_viz", 10);

        control_output_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "control_output", 
            10, 
            std::bind(&PendulumSimulator::control_output_callback, this, std::placeholders::_1)
        );

        start_time_ = this->now();
        timer_ = this->create_wall_timer(10ms, std::bind(&PendulumSimulator::control_output_callback_timer, this));
    }

private:
    // Pendulum parameters
    const double g = 9.81;  // Gravity constant
    const double L = 1.0;   // Pendulum length
    const double m = 1.0;   // Pendulum mass

    // Time parameters
    const double dt = 0.01; // Time step

    // This function calculates the next state (theta, omega) given the current state and the torque
    std::pair<double, double> pendulum_dynamics(double theta, double omega, double T) {
        // Update theta and omega using Euler's method
        double theta_next = theta + dt * omega;
        double omega_next = omega + dt * (-g/L * sin(theta) + T/m*pow(L,2));

        return {theta_next, omega_next};
    }

    void control_output_callback_timer() {
        // Create a dummy wrench message for now, you might want to replace this
        // with the actual wrench you're using in your project
        auto msg = std::make_shared<geometry_msgs::msg::Wrench>();
        msg->torque.z = 0.0;
        control_output_callback(msg);
    }

    void control_output_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        // Apply the pendulum dynamics
        auto [next_angle, next_angular_velocity] = pendulum_dynamics(current_angle_, current_angular_velocity_, msg->torque.z);

        current_angle_ = next_angle;
        current_angular_velocity_ = next_angular_velocity_;

        // Rest of your code...
    }

    rclcpp::Publisher<pendulum_control::msg::PendulumState>::SharedPtr control_input_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pendulum_viz_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr control_output_subscription_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    double current_angle_;
    double current_angular_velocity_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PendulumSimulator>());
    rclcpp::shutdown();
    return 0;
}
