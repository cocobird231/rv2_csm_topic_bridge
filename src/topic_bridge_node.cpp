/**
 * @file   topic_bridge_node.cpp
 * @brief  Composable ROS 2 node that subscribes to a configurable topic and
 *         bridges incoming messages into the rv2 ControlSignalManager (CSM)
 *         as a control signal source.
 *
 * This node acts as a generic "topic → CSM source" adapter.  It subscribes to
 * an arbitrary ROS 2 topic whose name and message type are specified at launch
 * time, and forwards each received message to the target ControlServer's CSM
 * via a ControlSignalSource.  No rate conversion is performed: every received
 * message is forwarded immediately, so the upstream publisher controls the
 * publish rate seen by the CSM.
 *
 * Supported message types (controlled by the `msg_type` parameter):
 *  - @c "joy"    → @c sensor_msgs::msg::Joy
 *  - @c "twist"  → @c geometry_msgs::msg::Twist
 *  - @c "string" → @c std_msgs::msg::String
 *
 * @par Parameters (see config/topic_bridge.yaml for defaults)
 *  - @b server_name                  Name of the target CSM server node.
 *  - @b csm_name                     Name of this node's own ControlSignalManager.
 *  - @b channel_name                 CSM channel name used for the source/sink pair.
 *  - @b topic_name                   ROS 2 topic to subscribe to (required).
 *  - @b msg_type                     Message type: "joy", "twist", or "string".
 *  - @b csm_mode                     CSM transport mode: "topic" (default) or "service".
 *                                    In topic mode the CSM source is a ROS publisher;
 *                                    in service mode it is a service client.
 *  - @b priority                     Source priority for CSM arbitration (1–100).
 *  - @b timeout_ms                   Inactivity timeout in milliseconds; the CSM
 *                                    marks the source TIMEOUT if no message arrives
 *                                    within this window.
 *  - @b disconnect_timeout_ms        Time in ms the source may stay in TIMEOUT before
 *                                    the CSM removes it automatically (0 = never).
 *  - @b csm_status_timer_interval_ms Period of the CSM status / disconnect timer (ms).
 *
 * @note Requires a multi-threaded executor (component_container_mt) because
 *       ControlSignalManager::registerSource() blocks waiting for a service
 *       response from the target CSM, and that response callback must be
 *       dispatched concurrently.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rv2_interfaces/msg/control_signal_const.hpp>
#include <rv2_interfaces/msg/control_signal_info.hpp>
#include "rv2_control_signal_transport/control_signal_manager.h"

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

using Joy    = sensor_msgs::msg::Joy;
using Twist  = geometry_msgs::msg::Twist;
using String = std_msgs::msg::String;
namespace CSC = rv2_interfaces::msg;
using rv2_interfaces::ControlSignalManager;


namespace rv2_csm_topic_bridge
{

/**
 * @brief Composable ROS 2 node that bridges a ROS 2 topic into the rv2
 *        ControlSignalManager as a control signal source.
 *
 * The node is intentionally lightweight at construction: it only declares and
 * reads parameters, creates the CSM (which immediately advertises its two
 * service servers), and schedules a 500 ms one-shot timer.  The timer fires
 * after the executor has started spinning and launches a deferred-init thread
 * that performs the blocking CSM source registration and creates the
 * subscription.
 */
class TopicBridgeNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct the node, declare all parameters, create the CSM, and
     *        schedule deferred initialisation.
     *
     * @param options  Component options forwarded from the composable-node
     *                 container (e.g. parameter overrides, remappings).
     */
    explicit TopicBridgeNode(const rclcpp::NodeOptions & options)
        : Node("topic_bridge", options)
    {
        // ── Declare & read parameters ───────────────────────────────────────────
        declare_parameter<std::string>("server_name",                  "control_server");
        declare_parameter<std::string>("csm_name",                     "topic_bridge");
        declare_parameter<std::string>("channel_name",                 "topic_bridge_control");
        declare_parameter<std::string>("topic_name",                   "");
        declare_parameter<std::string>("msg_type",                     "joy");
        declare_parameter<std::string>("csm_mode",                     "topic");
        declare_parameter<int64_t>    ("priority",                     50LL);
        declare_parameter<int64_t>    ("timeout_ms",                   2000LL);
        declare_parameter<int64_t>    ("disconnect_timeout_ms",        10000LL);
        declare_parameter<int64_t>    ("csm_status_timer_interval_ms", 1000LL);

        serverName_   = get_parameter("server_name").as_string();
        csmName_      = get_parameter("csm_name").as_string();
        channelName_  = get_parameter("channel_name").as_string();
        topicName_    = get_parameter("topic_name").as_string();
        msgType_      = get_parameter("msg_type").as_string();
        priority_     = static_cast<int8_t>(get_parameter("priority").as_int());
        timeoutNs_    = get_parameter("timeout_ms").as_int() * 1'000'000LL;
        disconnectNs_ = get_parameter("disconnect_timeout_ms").as_int() * 1'000'000LL;
        const int64_t csmStatusMs = get_parameter("csm_status_timer_interval_ms").as_int();

        {
            const std::string m = get_parameter("csm_mode").as_string();
            if (m == CSC::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE)
                csmMode_ = CSC::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE;
            else
                csmMode_ = CSC::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC;
        }

        // ── Validate required parameters ───────────────────────────────────────
        if (topicName_.empty()) {
            RCLCPP_ERROR(get_logger(),
                "[TopicBridge] Parameter 'topic_name' is required but is empty. "
                "Node will remain idle. Set it in the config file or as a "
                "launch argument.");
            return;
        }

        const bool validType =
            (msgType_ == CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY)    ||
            (msgType_ == CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST)   ||
            (msgType_ == CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_STRING);

        if (!validType) {
            RCLCPP_ERROR(get_logger(),
                "[TopicBridge] Unsupported msg_type '%s'. "
                "Supported values: \"joy\", \"twist\", \"string\". "
                "Node will remain idle.",
                msgType_.c_str());
            return;
        }

        // Service mode is not supported for "string" (no service type registered).
        if (csmMode_ == CSC::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE &&
            msgType_ == CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_STRING) {
            RCLCPP_ERROR(get_logger(),
                "[TopicBridge] csm_mode 'service' is not supported for msg_type 'string'. "
                "Node will remain idle.");
            return;
        }

        // ── CSM: advertises its two service servers immediately ────────────────
        csm_ = std::make_unique<ControlSignalManager>(this, csmName_, csmStatusMs);

        // ── Log all parameters ─────────────────────────────────────────────────
        {
            const auto names  = list_parameters({}, 0).names;
            const auto params = get_parameters(names);
            std::string log = "[TopicBridge] Parameters:";
            for (const auto & p : params)
                log += "\n  " + p.get_name() + " = " + p.value_to_string();
            RCLCPP_INFO(get_logger(), "%s", log.c_str());
        }

        // ── Deferred init ──────────────────────────────────────────────────────
        // registerSource() makes a blocking service call; it must execute after
        // the executor is spinning and must not block an executor thread.
        // A 500 ms one-shot timer guarantees the executor is running before the
        // init std::thread is launched.
        initTimer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                initTimer_->cancel();
                initThread_ = std::thread([this]() { _init(); });
            });
    }

    /**
     * @brief Destroy the node, signalling all callbacks to stop and joining
     *        the deferred-init thread.
     */
    ~TopicBridgeNode()
    {
        exitF_.store(true);
        // Clear subscriptions before joining so any in-flight callback can
        // check exitF_ and return early.
        joySub_.reset();
        twistSub_.reset();
        strSub_.reset();
        if (initThread_.joinable())
            initThread_.join();
    }

private:
    // ── Deferred initialisation (runs in initThread_) ─────────────────────────

    /**
     * @brief Register the CSM source with the target server and create the
     *        subscription for the configured message type.
     *
     * Runs in @c initThread_ to avoid blocking the ROS executor.
     * Returns early without creating a subscription if registration fails.
     */
    void _init()
    {
        if (!_registerSource(channelName_, msgType_)) {
            RCLCPP_ERROR(get_logger(),
                "[TopicBridge] Failed to register %s source for channel '%s' "
                "with server '%s'. Node will remain idle.",
                msgType_.c_str(), channelName_.c_str(), serverName_.c_str());
            return;
        }

        // Create the typed subscription and forward each message immediately
        // to the CSM source via sendErased().
        if (msgType_ == CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY) {
            joySub_ = create_subscription<Joy>(
                topicName_, rclcpp::QoS(10),
                [this](Joy::ConstSharedPtr msg) { _forward(msg.get()); });

        } else if (msgType_ == CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST) {
            twistSub_ = create_subscription<Twist>(
                topicName_, rclcpp::QoS(10),
                [this](Twist::ConstSharedPtr msg) { _forward(msg.get()); });

        } else {
            // "string"
            strSub_ = create_subscription<String>(
                topicName_, rclcpp::QoS(10),
                [this](String::ConstSharedPtr msg) { _forward(msg.get()); });
        }

        RCLCPP_INFO(get_logger(),
            "[TopicBridge] Ready. Subscribed to '%s' (%s) \u2192 channel '%s' on server '%s'.",
            topicName_.c_str(), msgType_.c_str(),
            channelName_.c_str(), serverName_.c_str());
    }

    /**
     * @brief Build a ControlSignalInfo descriptor and call
     *        ControlSignalManager::registerSource() on the target CSM.
     *
     * @param channel  CSM channel name to register.
     * @param type     ControlSignalConst type string ("joy", "twist", "string").
     * @return @c true on successful registration; @c false otherwise.
     */
    bool _registerSource(const std::string & channel, const std::string & type)
    {
        CSC::ControlSignalInfo info;
        info.target_csm_name        = serverName_;
        info.control_signal_mode    = csmMode_;
        info.control_signal_type    = type;
        info.channel_name           = channel;
        // send_freq_hz = 0: no fixed publish rate — the upstream topic publisher
        // controls the rate; the CSM uses timeout_ns for liveness detection only.
        info.send_freq_hz           = 0.0f;
        info.timeout_ns             = timeoutNs_;
        info.disconnect_timeout_ns  = disconnectNs_;
        info.use_keep_alive         = false;
        info.keep_alive_interval_ns = 0;
        info.priority               = priority_;

        const bool ok = csm_->registerSource(info, /*timeoutMs=*/5000);
        if (ok) {
            RCLCPP_INFO(get_logger(),
                "[TopicBridge] Registered %-6s source: ch='%s' \u2192 server='%s'",
                type.c_str(), channel.c_str(), serverName_.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "[TopicBridge] Could not register %s source with server '%s'.",
                type.c_str(), serverName_.c_str());
        }
        return ok;
    }

    /**
     * @brief Type-erased forward helper: passes a received message pointer to
     *        the CSM source's sendErased() method.
     *
     * @param msg  Pointer to the received message (must match the registered type).
     */
    void _forward(const void * msg)
    {
        if (exitF_.load()) return;
        auto src = csm_->getSource(channelName_);
        if (!src) return;
        bool cmdOk = false;
        src->sendErased(msg, cmdOk);
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    std::string serverName_;
    std::string csmName_;
    std::string channelName_;
    std::string topicName_;
    std::string msgType_;
    std::string csmMode_;
    int8_t      priority_{50};
    int64_t     timeoutNs_{2'000'000'000LL};
    int64_t     disconnectNs_{10'000'000'000LL};

    // ── CSM ───────────────────────────────────────────────────────────────────
    std::unique_ptr<ControlSignalManager> csm_;

    // ── Subscriptions (at most one is active at a time) ───────────────────────
    rclcpp::Subscription<Joy>::SharedPtr    joySub_;
    rclcpp::Subscription<Twist>::SharedPtr  twistSub_;
    rclcpp::Subscription<String>::SharedPtr strSub_;

    // ── Deferred init ─────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr initTimer_;
    std::thread                  initThread_;
    std::atomic<bool>            exitF_{false};
};

}  // namespace rv2_csm_topic_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(rv2_csm_topic_bridge::TopicBridgeNode)
