#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

using namespace std::chrono_literals;

class ControlOutputPublisher : public rclcpp::Node
{
public:
  ControlOutputPublisher() : Node("control_output_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("control_output", 10);
    timer_ = this->create_wall_timer(
        10ms, std::bind(&ControlOutputPublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = geometry_msgs::msg::Wrench();
    message.force.x = 0.0;
    message.force.y = 0.0;
    message.force.z = 0.0;
    message.torque.x = 0.0;
    message.torque.y = 0.0;
    message.torque.z = 15.0;

    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.torque.z);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlOutputPublisher>());
  rclcpp::shutdown();
  return 0;
}
