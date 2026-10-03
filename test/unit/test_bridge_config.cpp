#include <functional>
#include <limits>
#include <vector>
#include <gtest/gtest.h>
#include "rv2_csm_topic_bridge/bridge_config.hpp"

using rv2_csm_topic_bridge::BridgeConfig;

TEST(BridgeConfig, DefaultsMatchR1JoystickDescriptor)
{
    const BridgeConfig config;
    const auto info = config.info();
    EXPECT_EQ(info.controller_name, "local_xbox");
    EXPECT_EQ(info.channel_name, "topic_bridge_control");
    EXPECT_EQ(info.target_manager_name, "control_server");
    EXPECT_EQ(info.type, "joy");
    EXPECT_EQ(info.mode, "topic");
    EXPECT_EQ(info.priority, 80);
    EXPECT_EQ(info.timeout_ns, 2'000'000'000);
    EXPECT_EQ(info.disconnect_timeout_ns, 10'000'000'000);
    EXPECT_EQ(config.topicName, "/joy");
    EXPECT_EQ(config.managerOptions().statusIntervalMs, 100);
    EXPECT_FALSE(config.managerOptions().retryPolicy.autoRetryInitial);
}

TEST(BridgeConfig, EmptyControllerUsesChannelIdentity)
{
    BridgeConfig config;
    config.controllerName.clear();
    config.channelName = "/local/handset";
    EXPECT_EQ(config.info().controller_name, "/local/handset");
}

TEST(BridgeConfig, SupportsAllFiveR1Bindings)
{
    for (const auto* type : {"joy", "twist", "string"})
        for (const auto* mode : {"topic", "service"})
        {
            BridgeConfig config;
            config.type = type;
            config.mode = mode;
            if (config.type == "string" && config.mode == "service")
                EXPECT_THROW(config.info(), std::invalid_argument);
            else
            {
                EXPECT_EQ(config.info().type, type);
                EXPECT_EQ(config.info().mode, mode);
            }
        }
}

TEST(BridgeConfig, ValidatesPriorityBeforeNarrowing)
{
    BridgeConfig config;
    for (const int64_t priority : {-1000, -1, 0, 101, 128, 256, 336})
    {
        config.priority = priority;
        EXPECT_THROW(config.info(), std::invalid_argument) << priority;
    }
    for (const int64_t priority : {1, 80, 94, 100})
    {
        config.priority = priority;
        EXPECT_EQ(config.info().priority, priority);
    }
}

TEST(BridgeConfig, RejectsInvalidTimingAndMissingNames)
{
    const std::vector<std::function<void(BridgeConfig&)>> invalid = {
        [](auto& c)
        {
            c.topicName.clear();
        },
        [](auto& c)
        {
            c.serverName.clear();
        },
        [](auto& c)
        {
            c.managerName.clear();
        },
        [](auto& c)
        {
            c.masterName.clear();
        },
        [](auto& c)
        {
            c.channelName.clear();
        },
        [](auto& c)
        {
            c.type = "unknown";
        },
        [](auto& c)
        {
            c.mode = "typo";
        },
        [](auto& c)
        {
            c.timeoutMs = -1;
        },
        [](auto& c)
        {
            c.disconnectTimeoutMs = -1;
        },
        [](auto& c)
        {
            c.disconnectTimeoutMs = c.timeoutMs;
        },
        [](auto& c)
        {
            c.disconnectTimeoutMs = c.timeoutMs - 1;
        },
        [](auto& c)
        {
            c.mode = "service";
            c.timeoutMs = 0;
        },
        [](auto& c)
        {
            c.inputTimeoutMs = 0;
        },
        [](auto& c)
        {
            c.inputTimeoutMs = -1;
        },
        [](auto& c)
        {
            c.registrationTimeoutMs = 0;
        },
        [](auto& c)
        {
            c.registrationTimeoutMs = 5001;
        },
        [](auto& c)
        {
            c.registrationRetryMs = 0;
        },
        [](auto& c)
        {
            c.registrationRetryMs = 60001;
        },
        [](auto& c)
        {
            c.timeoutMs = std::numeric_limits<int64_t>::max();
        },
        [](auto& c)
        {
            c.disconnectTimeoutMs = std::numeric_limits<int64_t>::max();
        },
        [](auto& c)
        {
            c.inputTimeoutMs = std::numeric_limits<int64_t>::max();
        },
    };
    for (size_t index = 0; index < invalid.size(); ++index)
    {
        BridgeConfig config;
        invalid[index](config);
        EXPECT_THROW(config.info(), std::invalid_argument) << "case " << index;
    }
}

TEST(BridgeConfig, PreservesDisabledThresholdsAndExactConversion)
{
    BridgeConfig config;
    config.timeoutMs = 0;
    config.disconnectTimeoutMs = 0;
    EXPECT_EQ(config.info().timeout_ns, 0);
    EXPECT_EQ(config.info().disconnect_timeout_ns, 0);
    config.timeoutMs = 17;
    EXPECT_EQ(config.info().timeout_ns, 17'000'000);
    config.timeoutMs = std::numeric_limits<int64_t>::max() / 1'000'000;
    EXPECT_EQ(config.info().timeout_ns, config.timeoutMs * 1'000'000);
}

TEST(BridgeConfig, EnforcesHeartbeatDeadlineAndMasterIdentity)
{
    BridgeConfig config;
    for (const int64_t interval : {-1, 0, 300, 1000})
    {
        config.statusIntervalMs = interval;
        EXPECT_THROW(config.managerOptions(), std::invalid_argument);
    }
    config.statusIntervalMs = 299;
    config.masterName = "my_master";
    EXPECT_EQ(config.managerOptions().masterName, "my_master");
    EXPECT_TRUE(config.managerOptions().validate().empty());
    config.masterName.clear();
    EXPECT_THROW(config.managerOptions(), std::invalid_argument);
}
