#pragma once
#include "utils.h"
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ClockPublisher : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr mPub_;
    rclcpp::TimerBase::SharedPtr mTimer_;
    rclcpp::Clock mClock_;
    rclcpp::Time mTime_;
    std::chrono::time_point<std::chrono::steady_clock> mInitTp_;

    void _pubTmCb();

public:
    ClockPublisher(const std::string& nodeName);

    ~ClockPublisher();
};
