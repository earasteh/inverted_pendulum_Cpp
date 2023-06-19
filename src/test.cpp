class PendulumSimulator : public rclcpp::Node
{
public:
    PendulumSimulator() : Node("pendulum_simulator")
    {
        launch_time_ = this->now();
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        control_input_publisher_ = this->create_publisher<pendulum_control::msg::PendulumState>("control_input", 10);
        pendulum_viz_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pendulum_viz", 10);

        control_output_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
            "control_output", 
            10, 
            std::bind(&PendulumSimulator::control_output_callback, this, std::placeholders::_1)
        );
        // Create a timer which fires every 100ms
        timer_ = this->create_wall_timer(100ms, std::bind(&PendulumSimulator::timer_callback, this));
    }

private:
    void control_output_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
        // This method gets called whenever a new message arrives on the control_output topic.
        // You can put any code here which needs to react to these messages.
    }
    
    void timer_callback()
    {
        // This method gets called every 100ms, regardless of when messages are received on the control_output topic.

        rclcpp::Time now = this->now();
        double elapsed_time = (now - launch_time_).seconds();
        
        double new_angle = sin(elapsed_time * 2 * M_PI / 5);

        // Rest of the code for updating pendulum position...
    }

    rclcpp::Publisher<pendulum_control::msg::PendulumState>::SharedPtr control_input_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pendulum_viz_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr control_output_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time launch_time_;
};
