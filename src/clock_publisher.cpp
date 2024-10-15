#include <rv2_package_example/clock_publisher.h>
#include <chrono>

void ClockPublisher::_pubTmCb()
{
    auto msg = std::make_unique<rosgraph_msgs::msg::Clock>();
    auto now_ns = (std::chrono::steady_clock::now() - mInitTp_).count();
    auto now = rclcpp::Time(now_ns, RCL_ROS_TIME);
    msg->clock = now;
    mPub_->publish(std::move(msg));
}

ClockPublisher::ClockPublisher(const std::string& nodeName) : rclcpp::Node(nodeName)
{
    mInitTp_ = std::chrono::steady_clock::now();
    mPub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    mTimer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ClockPublisher::_pubTmCb, this));
    RCLCPP_INFO(this->get_logger(), "[ClockPublisher] Constructed");
}

ClockPublisher::~ClockPublisher()
{
    RCLCPP_INFO(this->get_logger(), "[ClockPublisher] Destroyed");
}
