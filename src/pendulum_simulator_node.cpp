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
    PendulumSimulator() : Node("pendulum_simulator"), current_angle_(45*M_PI/180), current_angular_velocity_(0)
    {
        //launch_time_ = this->now();
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        control_input_publisher_ = this->create_publisher<pendulum_control::msg::PendulumState>("control_input", 10);
        pendulum_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pendulum_viz", 10);

        control_output_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "control_output", 
            10, 
            std::bind(&PendulumSimulator::control_output_callback, this, std::placeholders::_1)
        );

        start_time_ = this->now(); // initialization time
        // Create a timer which fires every 1ms
        timer_ = this->create_wall_timer(1ms, std::bind(&PendulumSimulator::simulation_dynamics, this));
    }

private:
    // Pendulum parameters
    const double g = 9.81;  // Gravity constant
    const double L = 1.0;   // Pendulum length
    const double m = 1.0;   // Pendulum mass

    // Time parameters
    const double dt = 0.001; // Time step

    // // TF broadcaster function
    // void transformStamp_broadcaster(){
    // // tf broadcasting
    //     geometry_msgs::msg::TransformStamped transformStamped;
    //     transformStamped.header.stamp = this->now();
    //     transformStamped.header.frame_id = "world";
    //     transformStamped.child_frame_id = "base_link";
    //     transformStamped.transform.translation.x = 0.0;
    //     transformStamped.transform.translation.y = 0.0;
    //     transformStamped.transform.translation.z = 0.0;
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, 0);
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();

    //     tf_broadcaster_->sendTransform(transformStamped);
    // }

    // This function calculates the next state (theta, omega) given the current state and the torque
    std::pair<double, double> pendulum_dynamics(double theta, double omega, double tau_) {
        // theta = angle of the pendulum, omega= angular velocity of the pendulum, tau_ = torque input of the motor

        // Update theta and omega using Euler's method
        double theta_next = theta + dt * omega;
        double omega_next = omega + dt * (-g/L * sin(theta) + tau_/m*pow(L,2));

        return {theta_next, omega_next};
    }

    void simulation_dynamics() {
        // This method gets called every 1ms, regardless of when messages are received on the control_output topic.
        rclcpp::Time now = this->now();
        double elapsed_time = (now - start_time_).seconds();
        // tf broadcaster
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transformStamped);

        // auto msg = std::make_shared<geometry_msgs::msg::Wrench>();        
        // msg->torque.z = 0.0;
        // control_output_callback(msg);
        // RCLCPP_INFO(this->get_logger(), "Receving: '%f'", msg->torque.z);

        // pendulum dynamics
        auto [next_angle, next_angular_velocity_] = pendulum_dynamics(current_angle_, current_angular_velocity_, msg->torque.z);

        current_angle_ = next_angle;
        current_angular_velocity_ = next_angular_velocity_;

        ///// VISUALIZATION

        // Create a marker array for visualization
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Create a marker representing the pendulum's tip
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0;   // Assuming unit length and base at origin
        marker.pose.position.y = L*sin(current_angle_);  
        marker.pose.position.z = L*-cos(current_angle_);
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker_array.markers.push_back(marker);

        // Create a marker for the rod
        visualization_msgs::msg::Marker rod_marker;
        rod_marker.header.frame_id = "base_link";
        rod_marker.id = 1;
        rod_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        rod_marker.action = visualization_msgs::msg::Marker::ADD;
        rod_marker.pose.position.x = 0;
        rod_marker.pose.position.y = L/2*sin(current_angle_); 
        rod_marker.pose.position.z = -L/2*cos(current_angle_); 

        // double rod_roll = 0, rod_pitch = 0, rod_yaw = current_angle_;  // Angle in radians
        tf2::Quaternion q_rod;
        q_rod.setRPY(current_angle_, 0, 0);

        rod_marker.pose.orientation.x = q_rod.x();
        rod_marker.pose.orientation.y = q_rod.y();
        rod_marker.pose.orientation.z = q_rod.z();
        rod_marker.pose.orientation.w = q_rod.w();
        rod_marker.scale.x = 0.05;
        rod_marker.scale.y = 0.05;
        rod_marker.scale.z = L;   // The length of the rod
        rod_marker.color.a = 1.0;
        rod_marker.color.r = 1.0;
        rod_marker.color.g = 1.0;
        rod_marker.color.b = 0.0;

        marker_array.markers.push_back(rod_marker);

        // Publish pendulum visualization
        pendulum_viz_publisher_->publish(marker_array);
    }


void control_output_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    // This method gets called whenever a new message arrives on the control_output topic.

    rclcpp::Time now = this->now();
    double t_ = (now - start_time_).seconds();
    /////////
    std::cout << "Callback Timer: " << t_ << std::endl;
    // Update pendulum state based on received wrench
    // Here, for simplicity, let's just assume the torque in the z-direction directly maps to the angle of the pendulum
    // In a realistic scenario, you would use more complex physics involving moment of inertia, gravitational force etc.

    // Create control input message with the new state
    pendulum_control::msg::PendulumState control_input_msg;
    control_input_msg.angle = current_angle_;
    control_input_msg.angular_velocity = current_angular_velocity_;

    // Publish control input for the controller
    control_input_publisher_->publish(control_input_msg);
}

    rclcpp::Publisher<pendulum_control::msg::PendulumState>::SharedPtr control_input_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pendulum_viz_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr control_output_subscription_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // tf object
    
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
