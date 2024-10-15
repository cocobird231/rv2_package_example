#include <rv2_package_example/clock_publisher.h>
#include <rv2_package_example/sim_ultrasound_node.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

class Params : public rclcpp::Node
{
private:
    std::string mNodeName_;
    std::string mId_;
    bool mUseSimTime_;

public:
    Params(const std::string& nodeName) : rclcpp::Node(nodeName)
    {
        this->getParam("nodeName", nodeName, mNodeName_, "Node name: ", false);
        this->getParam("id", (std::string)"0", mId_, "Node ID: ", false);
        this->getParam("use_sim_time", false, mUseSimTime_, "Use sim time: ", false, true);
    }

    template<typename T>
    void getParam(std::string paramName, T defValue, T & outVal, std::string log_info, bool dynamic, bool noDeclare = false)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = !dynamic;

        if (!noDeclare)
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

    std::string getNodeName() const
    {
        return mNodeName_;
    }

    std::string getId() const
    {
        return mId_;
    }

    bool getUseSimTime() const
    {
        return mUseSimTime_;
    }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string nodeName = "sim_ultrasound_node";
    std::string id = "0";
    bool use_sim_time = false;
    {
        auto tmp = std::make_shared<Params>("tmp_paramsnode");
        nodeName = tmp->getNodeName();
        id = tmp->getId();
        use_sim_time = tmp->getUseSimTime();
    }

    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto node = std::make_shared<SimUltrasoundNode>(nodeName + "_" + id);
    exec->add_node(node);

    std::shared_ptr<ClockPublisher> clockNode;
    if (use_sim_time)
    {
        clockNode = std::make_shared<ClockPublisher>("clock_publisher");
        exec->add_node(clockNode);
    }

    exec->spin();// Blocking

    rclcpp::shutdown();
    return 0;
}
