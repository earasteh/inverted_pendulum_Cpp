#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <geometry_msgs/msg/wrench.hpp>
#include "pendulum_control/msg/pendulum_state.hpp"

using namespace std::chrono_literals;

class PendulumController : public rclcpp::Node
{
public:
    PendulumController() : Node("pendulum_controller")
    {
        //parameters filled with yaml config file
        this->declare_parameter<double>("K_theta", 30.0);
        this->declare_parameter<double>("K_omega", 30.0);
        K_theta = this->get_parameter("K_theta").as_double();
        K_omega = this->get_parameter("K_omega").as_double();

        publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("control_output", 10);
        subscription_ = this->create_subscription<pendulum_control::msg::PendulumState>(
            "control_input", 10, std::bind(&PendulumController::pendulum_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
        10ms, std::bind(&PendulumController::publish_message, this)); //controller runs at 10 ms
    }

private:
    void pendulum_callback(const pendulum_control::msg::PendulumState::SharedPtr msg)
    {
        // Process the incoming pendulum state and decide on the torque to apply
        torque_ = compute_torque(*msg); //controller
    }

    void publish_message()
    {
        // Create a Wrench message for the control output and set the torque z value
        auto message = geometry_msgs::msg::Wrench();
        message.force.x = 0.0;
        message.force.y = 0.0;
        message.force.z = 0.0;
        message.torque.x = 0.0;
        message.torque.y = 0.0;
        message.torque.z = torque_;

        // Publish the message
        RCLCPP_INFO(this->get_logger(), "Torque : '%f'", message.torque.z);
        publisher_->publish(message);
    }

    double compute_torque(const pendulum_control::msg::PendulumState& pendulum_state)
    {
        RCLCPP_INFO(this->get_logger(), "theta (deg) : '%f'", (M_PI-pendulum_state.angle) * 180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "omega (rad/sec) : '%f'", pendulum_state.angular_velocity);
        //control implementation (LQR)
        //double output = 35.97 * (M_PI - pendulum_state.angle) - 32.74 * pendulum_state.angular_velocity;
        //double output = 90.6700 * (M_PI - pendulum_state.angle) - 34.3706 * pendulum_state.angular_velocity;
        double output = K_theta * (M_PI - pendulum_state.angle) - K_omega * pendulum_state.angular_velocity;

        return output;
    }

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    rclcpp::Subscription<pendulum_control::msg::PendulumState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    double torque_, K_theta, K_omega;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PendulumController>());
    rclcpp::shutdown();
    return 0;
}
