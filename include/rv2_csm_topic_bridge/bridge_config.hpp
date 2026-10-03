#ifndef RV2_CSM_TOPIC_BRIDGE_BRIDGE_CONFIG_HPP
#define RV2_CSM_TOPIC_BRIDGE_BRIDGE_CONFIG_HPP

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "rv2_control_signal_transport/r1/control_signal_manager.h"

namespace rv2_csm_topic_bridge
{
/** Parameters at the boundary between a ROS topic and the R1 transport. */
struct BridgeConfig
{
    std::string serverName{"control_server"};
    std::string managerName{"topic_bridge"};
    std::string masterName{"csm_master"};
    std::string channelName{"topic_bridge_control"};
    std::string controllerName{"local_xbox"};
    std::string topicName{"/joy"};
    std::string type{"joy"};
    std::string mode{"topic"};
    int64_t priority{80};
    int64_t timeoutMs{2000};
    int64_t disconnectTimeoutMs{10000};
    int64_t statusIntervalMs{100};
    int64_t inputTimeoutMs{250};
    int64_t registrationTimeoutMs{1000};
    int64_t registrationRetryMs{200};

    /** Validate before narrowing integer parameters or allocating ROS entities. */
    rv2_interfaces::r1::ControlSignalInfo info() const
    {
        namespace r1 = rv2_interfaces::r1;
        constexpr auto maxMs = std::numeric_limits<int64_t>::max() / 1'000'000;
        if (priority < 1 || priority > 100)
            throw std::invalid_argument("priority must be in [1, 100]");
        if (timeoutMs < 0 || timeoutMs > maxMs || disconnectTimeoutMs < 0 || disconnectTimeoutMs > maxMs)
            throw std::invalid_argument("timeout_ms / disconnect_timeout_ms outside nanosecond range");
        if (topicName.empty() || managerName.empty() || masterName.empty())
            throw std::invalid_argument("topic_name, csm_name and master_name must not be empty");
        if (inputTimeoutMs <= 0 || inputTimeoutMs > maxMs)
            throw std::invalid_argument("input_timeout_ms must be positive and fit nanoseconds");
        if (registrationTimeoutMs <= 0 || registrationTimeoutMs > 5000)
            throw std::invalid_argument("registration_timeout_ms must be in [1, 5000]");
        if (registrationRetryMs <= 0 || registrationRetryMs > 60000)
            throw std::invalid_argument("registration_retry_ms must be in [1, 60000]");
        if (type != r1::kTypeJoy && type != r1::kTypeTwist && type != r1::kTypeString)
            throw std::invalid_argument("msg_type must be joy, twist or string");
        if (type == r1::kTypeString && mode == r1::kModeService)
            throw std::invalid_argument("string supports topic mode only");
        r1::ControlSignalInfo descriptor;
        descriptor.controller_name = controllerName.empty() ? channelName : controllerName;
        descriptor.channel_name = channelName;
        descriptor.target_manager_name = serverName;
        descriptor.mode = mode;
        descriptor.type = type;
        descriptor.priority = static_cast<int8_t>(priority);
        descriptor.timeout_ns = timeoutMs * 1'000'000;
        descriptor.disconnect_timeout_ns = disconnectTimeoutMs * 1'000'000;
        const auto validation = r1::validateControlSignalInfo(descriptor);
        if (!validation.valid)
            throw std::invalid_argument(validation.error);
        return descriptor;
    }

    /** Explicit deployment policy: live input drives initial registration retry. */
    rv2_interfaces::r1::ManagerOptions managerOptions() const
    {
        // The default 600 ms CSM heartbeat timeout requires ticks below 300 ms.
        if (statusIntervalMs <= 0 || statusIntervalMs >= 300)
            throw std::invalid_argument("csm_status_timer_interval_ms must be in [1, 299]");
        auto retry = rv2_interfaces::r1::RetryPolicy::Recommended();
        retry.autoRetryInitial = false;
        rv2_interfaces::r1::ManagerOptions options(retry);
        options.statusIntervalMs = statusIntervalMs;
        options.masterName = masterName;
        const auto error = options.validate();
        if (!error.empty())
            throw std::invalid_argument(error);
        return options;
    }
};
}  // namespace rv2_csm_topic_bridge
#endif
