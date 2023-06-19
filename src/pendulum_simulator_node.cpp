#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pendulum_control/msg/pendulum_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <random> //for std::random_device and std::mt19937
#include <chrono> //for std::chrono::system_clock

using namespace std::chrono_literals;

class PendulumSimulator : public rclcpp::Node
{
public:
    PendulumSimulator() : Node("pendulum_simulator"), current_angle_(45*M_PI/180), current_angular_velocity_(0.0), torque(0.0)
    {
        //parameters:
        this->declare_parameter<double>("m", 1.0);
        this->declare_parameter<double>("g", 9.81);
        this->declare_parameter<double>("L", 1.0);
        this->declare_parameter<std::string>("integrator", "Euler");
        this->declare_parameter<std::string>("sim_mode", "free-fall");
        
        m = this->get_parameter("m").as_double();
        g = this->get_parameter("g").as_double();
        L = this->get_parameter("L").as_double();
        sim_mode = this->get_parameter("sim_mode").as_string();
        integrator_type = this->get_parameter("integrator").as_string();

        // Initialize the random number generator with current time as seed
        std::random_device rd;
        generator.seed(rd());

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        control_input_publisher_ = this->create_publisher<pendulum_control::msg::PendulumState>("control_input", 10);
        pendulum_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pendulum_viz", 10);

        control_output_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "control_output", 
            10, 
            std::bind(&PendulumSimulator::control_output_callback, this, std::placeholders::_1)
        );

        start_time_ = this->now(); // initialization time
        last_time_ = this->get_clock()->now();
        // Create a timer which fires every 1ms
        timer_ = this->create_wall_timer(1ms, std::bind(&PendulumSimulator::simulation_spinner, this));
    }

private:
    // Pendulum parameters
    double g;  // Gravity constant
    double L;   // Pendulum length
    double m;   // Pendulum mass

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
    std::pair<double, double> pendulum_dynamics(double theta, double omega, double tau_, double dt) {
        // theta = angle of the pendulum, omega= angular velocity of the pendulum, tau_ = torque input of the motor
        double theta_next = 0, omega_next = 0;
        if(integrator_type == "Euler"){
            // Update theta and omega using Euler's method
            theta_next = theta + dt * omega;
            omega_next = omega + dt * (-g/L * sin(theta) + tau_/m*pow(L,2));
        }
        else if (integrator_type == "RK4"){ 
            //Update theta and omega using Runge-Kutta 4-th order
            auto f_omega = [this, tau_](double theta) {
            return - this->g/this->L * sin(theta) + tau_/this->m*pow(this->L,2);
            };
            auto f_theta = [](double omega) {
                return omega;
            };

            // RK4 for theta
            double k1_theta = dt * f_theta(omega);
            double k2_theta = dt * f_theta(omega + k1_theta / 2.0);
            double k3_theta = dt * f_theta(omega + k2_theta / 2.0);
            double k4_theta = dt * f_theta(omega + k3_theta);
            theta_next = theta + (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta) / 6.0;

            // RK4 for omega
            double k1_omega = dt * f_omega(theta);
            double k2_omega = dt * f_omega(theta + k1_theta / 2.0);
            double k3_omega = dt * f_omega(theta + k2_theta / 2.0);
            double k4_omega = dt * f_omega(theta + k3_theta);
            omega_next = omega + (k1_omega + 2 * k2_omega + 2 * k3_omega + k4_omega) / 6.0;
        }

        return {theta_next, omega_next};
    }

    void simulation_spinner() {
        // This method gets called every 1ms, regardless of when messages are received on the control_output topic.
        
        // calculate dt
        auto current_time = this->get_clock()->now();
        auto dt = (current_time - last_time_).seconds(); // actual time since last call in seconds
        last_time_ = current_time;
        
        
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

        if(sim_mode == "free-fall"){
            // ignore the controller
            torque = 0;
        }
        else if(sim_mode == "perturb"){
            //perturbation
            torque += generate_perturbation();
        }
        else{
            // controlled torque
        }

        // pendulum dynamics
        auto [next_angle, next_angular_velocity_] = pendulum_dynamics(current_angle_, current_angular_velocity_, torque, dt);

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

    // update motor torque based on the received values from the controller node
    if(sim_mode != "free-fall"){
        torque = msg->torque.z;
    }
    else{ // free-fall
        torque = 0;
    }

    
    // Create control input message with the new state
    pendulum_control::msg::PendulumState control_input_msg;
    control_input_msg.angle = current_angle_;
    control_input_msg.angular_velocity = current_angular_velocity_;

    // Publish control input for the controller (pendulum states)
    control_input_publisher_->publish(control_input_msg);
}

double generate_perturbation() {
    std::normal_distribution<double> distribution(5, 5); // 5 N.m mean
    return distribution(generator);
}

    std::string sim_mode;
    std::string integrator_type;

    std::mt19937 generator;

    rclcpp::Publisher<pendulum_control::msg::PendulumState>::SharedPtr control_input_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pendulum_viz_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr control_output_subscription_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // tf object
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_, last_time_; //start_time_ is beginning of simulation, last_time_ is last timestamp that the simulation_spinner() ran
    double current_angle_;
    double current_angular_velocity_;
    double torque;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PendulumSimulator>());
    rclcpp::shutdown();
    return 0;
}
