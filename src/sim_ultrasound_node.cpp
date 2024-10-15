#include <rv2_package_example/sim_ultrasound_node.h>
#include <chrono>

void SimUltrasoundNode::_pubTmCb()
{
    for (auto & [iName, iPub] : mPubs_)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Range>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = iName;
        msg->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        msg->field_of_view = 0.1;
        msg->min_range = 0.02;
        msg->max_range = 4.0;
        msg->range = 0.02 + 3.98 * mGen_() / mGen_.max();
        iPub->publish(std::move(msg));
    }
}

SimUltrasoundNode::SimUltrasoundNode(const std::string& nodeName) : 
    rclcpp::Node(nodeName), 
    mGen_(mRd_())
{
    std::string topicName = "topic";
    std::vector<std::string> topicNameSuffixVec;
    int pubNum = 1;
    double pubPeriod_ms = 500.0;
    this->getParam("topicName", topicName, topicName, "Topic name: ", false);
    this->getParam("topicNameSuffixList", topicNameSuffixVec, topicNameSuffixVec, "Topic suffix: ", false);
    this->getParam("publisherCnt", pubNum, pubNum, "Publisher count: ", false);
    this->getParam("publishPeriod_ms", pubPeriod_ms, pubPeriod_ms, "Pub period ms: ", false);

    if (pubNum > topicNameSuffixVec.size() || pubNum < 1)
    {
        RCLCPP_WARN(this->get_logger(), "Publisher count is greater than topic suffix count or less than 1, set publisher count to topic suffix count: %ld", topicNameSuffixVec.size());
        pubNum = topicNameSuffixVec.size();
    }
    for (int i = 0; i < pubNum; i++)
    {
        std::string pubName = topicName + topicNameSuffixVec[i];
        mPubs_[pubName] = this->create_publisher<sensor_msgs::msg::Range>(pubName, 10);
    }
    mTimer_ = this->create_wall_timer(std::chrono::duration<double, std::milli>(pubPeriod_ms), std::bind(&SimUltrasoundNode::_pubTmCb, this));
    RCLCPP_INFO(this->get_logger(), "[SimUltrasoundNode] Constructed");
}

SimUltrasoundNode::~SimUltrasoundNode()
{
    RCLCPP_INFO(this->get_logger(), "[SimUltrasoundNode] Destroyed");
}

template<typename T>
void SimUltrasoundNode::getParam(std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = !dynamic;

    declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

    if (!get_parameter(paramName, outVal))
    {
        RCLCPP_WARN_STREAM(get_logger(),
            "The parameter '"
            << paramName
            << "' is not available or is not valid, using the default value: "
            << GenSS(defValue).str());
    }

    if (!log_info.empty())
    {
        RCLCPP_INFO_STREAM(get_logger(), log_info << GenSS(outVal).str());
    }
}
