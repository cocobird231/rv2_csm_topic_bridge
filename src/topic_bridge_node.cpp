#include "rv2_csm_topic_bridge/topic_bridge_node.hpp"
#include "rv2_csm_topic_bridge/bridge_config.hpp"
#include "rv2_csm_topic_bridge/input_mailbox.hpp"
#include <exception>
#include <thread>
#include <rclcpp_components/register_node_macro.hpp>

namespace rv2_csm_topic_bridge
{
namespace r1 = rv2_interfaces::r1;
using namespace std::chrono_literals;
namespace
{
BridgeConfig readConfig(rclcpp::Node& node)
{
    BridgeConfig config;
    config.serverName = node.declare_parameter("server_name", config.serverName);
    config.managerName = node.declare_parameter("csm_name", config.managerName);
    config.masterName = node.declare_parameter("master_name", config.masterName);
    config.channelName = node.declare_parameter("channel_name", config.channelName);
    config.controllerName = node.declare_parameter("controller_name", config.controllerName);
    config.topicName = node.declare_parameter("topic_name", config.topicName);
    config.type = node.declare_parameter("msg_type", config.type);
    config.mode = node.declare_parameter("csm_mode", config.mode);
    config.priority = node.declare_parameter("priority", config.priority);
    config.timeoutMs = node.declare_parameter("timeout_ms", config.timeoutMs);
    config.disconnectTimeoutMs = node.declare_parameter("disconnect_timeout_ms", config.disconnectTimeoutMs);
    config.statusIntervalMs = node.declare_parameter("csm_status_timer_interval_ms", config.statusIntervalMs);
    config.inputTimeoutMs = node.declare_parameter("input_timeout_ms", config.inputTimeoutMs);
    config.registrationTimeoutMs = node.declare_parameter("registration_timeout_ms", config.registrationTimeoutMs);
    config.registrationRetryMs = node.declare_parameter("registration_retry_ms", config.registrationRetryMs);
    return config;
}
}  // namespace

struct TopicBridgeNode::Impl
{
    explicit Impl(TopicBridgeNode& node) :
        config(readConfig(node)),
        descriptor(config.info()),
        logger(node.get_logger()),
        inbox(std::make_shared<InputMailbox>()),
        manager(std::make_unique<r1::ControlSignalManager>(&node, config.managerName, config.managerOptions()))
    {
        for (const auto state :
             {r1::ControlSignalState::ACTIVE, r1::ControlSignalState::TIMEOUT, r1::ControlSignalState::DISCONNECTED})
        {
            manager->registerSourceStateCallback(
                state,
                [log = logger](const std::string& controller, r1::ControlSignalState, r1::ControlSignalState next)
                {
                    const char* label = next == r1::ControlSignalState::ACTIVE    ? "ACTIVE"
                                        : next == r1::ControlSignalState::TIMEOUT ? "TIMEOUT"
                                                                                  : "DISCONNECTED";
                    RCLCPP_INFO(log, "R1 source '%s': %s", controller.c_str(), label);
                });
        }
        // Best-effort requested reliability matches reliable and sensor-data publishers.
        const auto qos = rclcpp::SensorDataQoS().keep_last(1);
        if (config.type == r1::kTypeJoy)
            subscription = node.create_subscription<sensor_msgs::msg::Joy>(
                config.topicName,
                qos,
                [mailbox = inbox](sensor_msgs::msg::Joy::ConstSharedPtr msg)
                {
                    mailbox->submit(std::move(msg));
                });
        else if (config.type == r1::kTypeTwist)
            subscription = node.create_subscription<geometry_msgs::msg::Twist>(
                config.topicName,
                qos,
                [mailbox = inbox](geometry_msgs::msg::Twist::ConstSharedPtr msg)
                {
                    mailbox->submit(std::move(msg));
                });
        else
            subscription = node.create_subscription<std_msgs::msg::String>(
                config.topicName,
                qos,
                [mailbox = inbox](std_msgs::msg::String::ConstSharedPtr msg)
                {
                    mailbox->submit(std::move(msg));
                });
        RCLCPP_INFO(logger,
                    "R1 bridge: %s (%s) -> %s, controller=%s priority=%d; waiting for live input",
                    config.topicName.c_str(),
                    config.type.c_str(),
                    config.serverName.c_str(),
                    descriptor.controller_name.c_str(),
                    static_cast<int>(descriptor.priority));
        worker = std::thread(
            [this]
            {
                run();
            });
    }

    ~Impl()
    {
        // A dispatched subscription callback retains only the independent mailbox.
        inbox->stop();
        subscription.reset();
        if (worker.joinable())
            worker.join();
        manager.reset();
    }

    void run()
    {
        r1::SourceHandle handle;
        uint64_t observed = 0;
        uint64_t consumed = 0;
        auto nextRegistration = InputClock::time_point::min();
        auto nextWarning = InputClock::time_point::min();
        const auto maxAge = std::chrono::milliseconds(config.inputTimeoutMs);
        try
        {
            while (!inbox->stopped())
            {
                auto sample = inbox->wait(observed, 20ms);
                if (!sample || sample->sequence == consumed || !sample->fresh(InputClock::now(), maxAge))
                    continue;
                if (!handle.valid())
                {
                    if (InputClock::now() < nextRegistration)
                        continue;
                    nextRegistration = InputClock::now() + std::chrono::milliseconds(config.registrationRetryMs);
                    const auto result = manager->registerSource(descriptor, config.registrationTimeoutMs);
                    handle = result.handle;
                    if (!handle.valid() && InputClock::now() >= nextWarning)
                    {
                        RCLCPP_WARN(logger,
                                    "Registration unavailable (R1 result=%u); retrying while input is fresh",
                                    static_cast<unsigned>(result.code));
                        nextWarning = InputClock::now() + 5s;
                    }
                }
                if (!handle.ready())
                    continue;
                // Blocking registration can outlive a command: read the mailbox again.
                sample = inbox->latest();
                if (!sample || sample->sequence == consumed || !sample->fresh(InputClock::now(), maxAge))
                    continue;
                const auto result = std::visit(
                    [&handle](const auto& msg)
                    {
                        return handle.send(*msg);
                    },
                    sample->message);
                // RETRYING issued no request. Never replay an uncertain service result.
                if (result != r1::SendResult::RETRYING)
                    consumed = sample->sequence;
                if (result != r1::SendResult::OK && result != r1::SendResult::RETRYING &&
                    InputClock::now() >= nextWarning)
                {
                    RCLCPP_WARN(logger, "R1 send result=%u; awaiting fresh input", static_cast<unsigned>(result));
                    nextWarning = InputClock::now() + 5s;
                }
            }
            if (handle.valid())
                manager->unregisterSource(handle);
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(logger, "R1 bridge worker stopped: %s", error.what());
            inbox->stop();
        }
    }
    BridgeConfig config;
    r1::ControlSignalInfo descriptor;
    rclcpp::Logger logger;
    std::shared_ptr<InputMailbox> inbox;
    std::unique_ptr<r1::ControlSignalManager> manager;
    rclcpp::SubscriptionBase::SharedPtr subscription;
    std::thread worker;
};

TopicBridgeNode::TopicBridgeNode(const rclcpp::NodeOptions& options) :
    Node("topic_bridge", options),
    impl_(std::make_unique<Impl>(*this))
{
}
TopicBridgeNode::~TopicBridgeNode() = default;
}  // namespace rv2_csm_topic_bridge
RCLCPP_COMPONENTS_REGISTER_NODE(rv2_csm_topic_bridge::TopicBridgeNode)
