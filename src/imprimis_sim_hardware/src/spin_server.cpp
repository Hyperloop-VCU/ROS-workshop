// Standard headers
#include <chrono>
#include <memory>
#include <string>
#include <atomic>

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/empty.hpp"

// This "using" section makes writing the code a lot easier
// For example, instead of writing "std_srvs::srv::Empty", I can just write "Empty"
using namespace std;
using namespace std::placeholders;
using rclcpp::Node;
using geometry_msgs::msg::TwistStamped;
using std_srvs::srv::Empty;



// NODE CLASS DEFINITION //
class SpinServiceNode : public Node
{
public:

  // Constructor; initialize node, declare spin_speed parameter, set spinEnabled to false, create all ROS objects
  SpinServiceNode() : Node("spin_service_node")
  {
    this->declare_parameter<double>("spin_speed", 0.5);
    spinEnabled = false;
    publisher_ = this->create_publisher<TwistStamped>("diffbot_base_controller/cmd_vel", 10);
    timer_ = this->create_wall_timer(50ms, bind(&SpinServiceNode::timer_cb, this));
    service_ = this->create_service<Empty>("spin_service", bind(&SpinServiceNode::service_cb, this, _1, _2));
  }
  

  // Service callback function; this code runs whenever the service is called.
  // It toggles the "spinEnabled" variable, which determines the contents of the published velocity command
  void service_cb(const shared_ptr<Empty::Request> request, shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spin Service called!");
    if (spinEnabled) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping the robot.");
        spinEnabled = false;
    }
    else {
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Beginning the spin."); 
       spinEnabled = true;
    }
  }
  

  // Timer callback function; this runs once every 50ms to publish the velocity command.
  // The command's angular velocity will be zero if spinEnabled is false, and whatever the spin_speed parameter is if spinEnabled is true.
  void timer_cb()
  {
    auto msg = TwistStamped();
    msg.header.stamp = get_clock()->now();
    if (spinEnabled) {
        msg.twist.angular.z = this->get_parameter("spin_speed").as_double();
    }
    else {
        msg.twist.angular.z = 0.0;
    }
    publisher_->publish(msg);
  }


// Internal variable definitions
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<TwistStamped>::SharedPtr publisher_;
  rclcpp::Service<Empty>::SharedPtr service_;
  atomic<bool> spinEnabled;
};
// END NODE CLASS DEFINITION //



// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<SpinServiceNode>());
  rclcpp::shutdown();
  return 0;
}