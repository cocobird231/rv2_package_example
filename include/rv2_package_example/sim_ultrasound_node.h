#pragma once
#include "utils.h"
#include <map>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/range.hpp>

class SimUltrasoundNode : public rclcpp::Node
{
private:
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> mPubs_;
    rclcpp::TimerBase::SharedPtr mTimer_;

    // Rand
    std::random_device mRd_;
    std::mt19937 mGen_;

private:
    void _pubTmCb();// Call by timer

public:
    SimUltrasoundNode(const std::string& nodeName);

    ~SimUltrasoundNode();

    template<typename T>
    void getParam(std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic);
};
