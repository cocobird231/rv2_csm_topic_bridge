#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <r1_interfaces/srv/control_signal_joy.hpp>
#include "rv2_csm_topic_bridge/topic_bridge_node.hpp"
#include "rv2_control_signal_transport/r1/control_signal_manager.h"
#include "rv2_control_signal_transport/r1/csm_master.h"

namespace
{
using namespace std::chrono_literals;
namespace r1 = rv2_interfaces::r1;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
using String = std_msgs::msg::String;
using Bridge = rv2_csm_topic_bridge::TopicBridgeNode;
using Manage = r1_interfaces::srv::ControlSignalManage;
using JoyService = r1_interfaces::srv::ControlSignalJoy;

bool waitFor(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 5s)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do
    {
        if (predicate())
            return true;
        std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
}

Joy joy(float value)
{
    Joy message;
    message.header.frame_id = "physical_joystick";
    message.header.stamp.sec = 123;
    message.header.stamp.nanosec = 456;
    message.axes = {value, -0.5f, 0.75f};
    message.buttons = {1, 0, 1};
    return message;
}

template <typename Message> struct Received
{
    void append(const Message& message, const r1::ControlSignalInfo& info)
    {
        std::lock_guard<std::mutex> lock(mutex);
        messages.push_back(message);
        descriptor = info;
    }
    size_t count()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return messages.size();
    }
    bool contains(const Message& expected)
    {
        std::lock_guard<std::mutex> lock(mutex);
        return std::find(messages.begin(), messages.end(), expected) != messages.end();
    }
    std::mutex mutex;
    std::vector<Message> messages;
    r1::ControlSignalInfo descriptor;
};

class BridgeIntegration : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override
    {
        static std::atomic<unsigned> next{0};
        prefix = "bridge_test_" + std::to_string(++next);
        inputTopic = "/" + prefix + "/input";
        channel = "/" + prefix + "/data";
        controller = prefix + "_controller";
        serverName = prefix + "_server";
        masterName = prefix + "_master";
        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
        publisherNode = std::make_shared<rclcpp::Node>(prefix + "_publisher");
        executor->add_node(publisherNode);
        spinning = std::thread(
            [this]
            {
                executor->spin();
            });
        ASSERT_TRUE(waitFor(
            [this]
            {
                return executor->is_spinning();
            }));
    }
    void TearDown() override
    {
        if (bridge)
        {
            executor->remove_node(bridge);
            bridge.reset();
        }
        executor->cancel();
        if (spinning.joinable())
            spinning.join();
        server.reset();
        master.reset();
        serverNode.reset();
        masterNode.reset();
        publisherNode.reset();
        executor.reset();
    }
    rclcpp::NodeOptions options(const std::string& type = "joy", const std::string& mode = "topic")
    {
        return rclcpp::NodeOptions()
            .append_parameter_override("topic_name", inputTopic)
            .append_parameter_override("server_name", serverName)
            .append_parameter_override("master_name", masterName)
            .append_parameter_override("csm_name", prefix + "_source")
            .append_parameter_override("controller_name", controller)
            .append_parameter_override("channel_name", channel)
            .append_parameter_override("msg_type", type)
            .append_parameter_override("csm_mode", mode)
            .append_parameter_override("timeout_ms", int64_t{200})
            .append_parameter_override("disconnect_timeout_ms", int64_t{900})
            .append_parameter_override("csm_status_timer_interval_ms", int64_t{25})
            .append_parameter_override("input_timeout_ms", int64_t{100})
            .append_parameter_override("registration_timeout_ms", int64_t{700})
            .append_parameter_override("registration_retry_ms", int64_t{100});
    }
    void startBridge(const rclcpp::NodeOptions& config)
    {
        bridge = std::make_shared<Bridge>(config);
        executor->add_node(bridge);
    }
    void startServer()
    {
        serverNode = std::make_shared<rclcpp::Node>(serverName);
        auto policy = r1::RetryPolicy::Recommended();
        r1::ManagerOptions config(policy);
        config.statusIntervalMs = 25;
        config.masterName = masterName;
        config.csmDisconnectTimeoutNs = 900'000'000;
        server = std::make_unique<r1::ControlSignalManager>(serverNode.get(), serverName, config);
        executor->add_node(serverNode);
    }
    void stopServer()
    {
        executor->remove_node(serverNode);
        server.reset();
        serverNode.reset();
    }
    void startMaster()
    {
        masterNode = std::make_shared<rclcpp::Node>(masterName);
        r1::MasterOptions config(r1::NotificationRetryPolicy(25, 200, 0.0, 8));
        config.masterName = masterName;
        config.tickIntervalMs = 25;
        config.pairGraceMs = 100;
        master = std::make_unique<r1::CsmMaster>(masterNode.get(), config);
        executor->add_node(masterNode);
    }
    template <typename Message> std::shared_ptr<Received<Message>> collect()
    {
        auto received = std::make_shared<Received<Message>>();
        EXPECT_TRUE(server->registerCallback<Message>(
            [received](const Message& message, const r1::ControlSignalInfo& info)
            {
                received->append(message, info);
            }));
        return received;
    }
    template <typename Message>
    bool publishUntil(const typename rclcpp::Publisher<Message>::SharedPtr& publisher,
                      const Message& message,
                      const std::function<bool()>& predicate,
                      std::chrono::milliseconds timeout = 5s)
    {
        return waitFor(
            [&]
            {
                publisher->publish(message);
                return predicate();
            },
            timeout);
    }
    template <typename Message>
    void publishFor(const typename rclcpp::Publisher<Message>::SharedPtr& publisher,
                    const Message& message,
                    std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            publisher->publish(message);
            std::this_thread::sleep_for(20ms);
        }
    }
    template <typename Message> void verifyBinding(const std::string& type, const std::string& mode, const Message& msg)
    {
        startServer();
        auto received = collect<Message>();
        startBridge(options(type, mode));
        auto publisher = publisherNode->create_publisher<Message>(inputTopic, rclcpp::SensorDataQoS());
        ASSERT_TRUE(publishUntil<Message>(publisher,
                                          msg,
                                          [&]
                                          {
                                              return received->contains(msg);
                                          }));
        ASSERT_TRUE(publishUntil<Message>(publisher,
                                          msg,
                                          [&]
                                          {
                                              return server->getSinkState(controller) == r1::ControlSignalState::ACTIVE;
                                          }));
        const auto handle = server->getSink(controller);
        ASSERT_TRUE(handle.valid());
        ASSERT_TRUE(handle.info());
        EXPECT_EQ(handle.info()->controller_name, controller);
        EXPECT_EQ(handle.info()->channel_name, channel);
        EXPECT_EQ(handle.info()->target_manager_name, serverName);
        EXPECT_EQ(handle.info()->priority, 80);
        EXPECT_EQ(handle.info()->type, type);
        EXPECT_EQ(handle.info()->mode, mode);
    }
    std::string prefix, inputTopic, channel, controller, serverName, masterName;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    rclcpp::Node::SharedPtr publisherNode, serverNode, masterNode;
    std::shared_ptr<Bridge> bridge;
    std::unique_ptr<r1::ControlSignalManager> server;
    std::unique_ptr<r1::CsmMaster> master;
    std::thread spinning;
};

TEST_F(BridgeIntegration, JoyTopicPreservesEveryFieldFromBestEffortPublisher)
{
    verifyBinding("joy", "topic", joy(0.25f));
}
TEST_F(BridgeIntegration, JoyServiceRunsOutsideCallbackGroupAndPreservesPayload)
{
    verifyBinding("joy", "service", joy(-0.25f));
}
TEST_F(BridgeIntegration, TwistTopicPreservesPayload)
{
    Twist message;
    message.linear.x = 0.12;
    message.linear.y = -0.34;
    message.angular.z = 0.56;
    verifyBinding("twist", "topic", message);
}
TEST_F(BridgeIntegration, TwistServicePreservesPayload)
{
    Twist message;
    message.linear.z = 0.42;
    message.angular.x = -0.31;
    verifyBinding("twist", "service", message);
}
TEST_F(BridgeIntegration, StringTopicPreservesPayload)
{
    String message;
    message.data = "operator command \xe6\xb8\xac\xe8\xa9\xa6";
    verifyBinding("string", "topic", message);
}

TEST_F(BridgeIntegration, SilenceTimesOutWithoutReplayAndFreshInputRecovers)
{
    startServer();
    auto received = collect<Joy>();
    startBridge(options());
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.1f),
                                  [&]
                                  {
                                      return received->contains(joy(0.1f));
                                  }));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return server->getSinkState(controller) == r1::ControlSignalState::TIMEOUT;
        },
        700ms));
    const auto count = received->count();
    std::this_thread::sleep_for(150ms);
    EXPECT_EQ(received->count(), count);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.9f),
                                  [&]
                                  {
                                      return received->contains(joy(0.9f));
                                  }));
    EXPECT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.9f),
                                  [&]
                                  {
                                      return server->getSinkState(controller) == r1::ControlSignalState::ACTIVE;
                                  }));
}

TEST_F(BridgeIntegration, PhysicalReconnectionRegistersAgainAfterTerminalInactivity)
{
    startServer();
    auto received = collect<Joy>();
    startBridge(options());
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.1f),
                                  [&]
                                  {
                                      return received->contains(joy(0.1f));
                                  }));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return !server->getSink(controller).valid();
        },
        3s));
    const auto count = received->count();
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(received->count(), count);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(-0.8f),
                                  [&]
                                  {
                                      return received->contains(joy(-0.8f));
                                  }));
    EXPECT_TRUE(server->getSink(controller).valid());
}

TEST_F(BridgeIntegration, ServerCanStartAfterMoreThanThreeInitialRetryAttempts)
{
    startBridge(options());
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return publisher->get_subscription_count() > 0;
        }));
    publishFor<Joy>(publisher, joy(0.1f), 2500ms);
    startServer();
    auto received = collect<Joy>();
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.8f),
                                  [&]
                                  {
                                      return received->contains(joy(0.8f));
                                  }));
}

TEST_F(BridgeIntegration, StaleInputDoesNotRegisterOrReplayWhenServerAppears)
{
    startBridge(options());
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return publisher->get_subscription_count() > 0;
        }));
    publishFor<Joy>(publisher, joy(0.1f), 200ms);
    std::this_thread::sleep_for(300ms);
    startServer();
    auto received = collect<Joy>();
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(received->count(), 0u);
    EXPECT_FALSE(server->getSink(controller).valid());
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.8f),
                                  [&]
                                  {
                                      return received->contains(joy(0.8f));
                                  }));
    EXPECT_FALSE(received->contains(joy(0.1f)));
}

TEST_F(BridgeIntegration, BlockingRegistrationCannotReplayAnExpiredInput)
{
    auto entered = std::make_shared<std::atomic<bool>>(false);
    auto management = publisherNode->create_service<Manage>(
        serverName + "/control_signal_manage",
        [entered](const Manage::Request::SharedPtr request, Manage::Response::SharedPtr response)
        {
            if (request->op == Manage::Request::OP_REGISTER)
            {
                entered->store(true);
                std::this_thread::sleep_for(400ms);
            }
            response->response = Manage::Response::RESPONSE_SUCCESS;
        });
    auto delivered = std::make_shared<std::atomic<unsigned>>(0);
    auto data = publisherNode->create_subscription<Joy>(channel,
                                                        10,
                                                        [delivered](Joy::ConstSharedPtr)
                                                        {
                                                            delivered->fetch_add(1);
                                                        });
    auto snapshots = std::make_shared<std::atomic<unsigned>>(0);
    auto registered = std::make_shared<std::atomic<bool>>(false);
    auto activity = std::make_shared<std::atomic<bool>>(false);
    auto status = publisherNode->create_subscription<r1_interfaces::msg::ManagerStatus>(
        prefix + "_source/status",
        10,
        [snapshots, registered, activity](r1_interfaces::msg::ManagerStatus::ConstSharedPtr message)
        {
            snapshots->fetch_add(1);
            for (const auto& entry : message->sources)
            {
                using Entry = r1_interfaces::msg::EntryStatus;
                if (entry.endpoint_present && entry.registration_phase == Entry::PHASE_REGISTERED)
                    registered->store(true);
                if (entry.state == Entry::STATE_ACTIVE || entry.state == Entry::STATE_TIMEOUT)
                    activity->store(true);
            }
        });
    startBridge(options());
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return publisher->get_subscription_count() > 0 && snapshots->load() > 0;
        }));
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.6f),
                                  [&]
                                  {
                                      return entered->load();
                                  }));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return registered->load();
        }));
    std::this_thread::sleep_for(250ms);
    EXPECT_EQ(delivered->load(), 0u);
    EXPECT_FALSE(activity->load()) << "Even an undiscovered DDS publish must not record stale source activity";
}

TEST_F(BridgeIntegration, ServiceWaitDoesNotBlockBridgeDefaultCallbackGroup)
{
    auto management =
        publisherNode->create_service<Manage>(serverName + "/control_signal_manage",
                                              [](const Manage::Request::SharedPtr, Manage::Response::SharedPtr response)
                                              {
                                                  response->response = Manage::Response::RESPONSE_SUCCESS;
                                              });
    auto entered = std::make_shared<std::atomic<bool>>(false);
    auto released = std::make_shared<std::atomic<bool>>(false);
    auto data = publisherNode->create_service<JoyService>(
        channel,
        [entered, released](const JoyService::Request::SharedPtr, JoyService::Response::SharedPtr response)
        {
            entered->store(true);
            waitFor(
                [&]
                {
                    return released->load();
                },
                1s);
            response->response = JoyService::Response::SRV_RES_SUCCESS;
        });
    startBridge(options("joy", "service")
                    .append_parameter_override("timeout_ms", int64_t{700})
                    .append_parameter_override("disconnect_timeout_ms", int64_t{2000}));
    auto ticks = std::make_shared<std::atomic<unsigned>>(0);
    auto progress = bridge->create_wall_timer(10ms,
                                              [ticks]
                                              {
                                                  ticks->fetch_add(1);
                                              });
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.5f),
                                  [&]
                                  {
                                      return entered->load();
                                  }));
    const auto before = ticks->load();
    const bool progressed = waitFor(
        [&]
        {
            return ticks->load() >= before + 3;
        },
        150ms);
    released->store(true);
    EXPECT_TRUE(progressed) << "Service wait must leave the subscription/default callback group free";
}

TEST_F(BridgeIntegration, MasterReconcilesRestartedServerWhileJoystickKeepsPublishing)
{
    startMaster();
    startServer();
    auto first = collect<Joy>();
    startBridge(options());
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.1f),
                                  [&]
                                  {
                                      return first->contains(joy(0.1f));
                                  }));
    publishFor<Joy>(publisher, joy(0.1f), 500ms);
    stopServer();
    publishFor<Joy>(publisher, joy(0.2f), 1200ms);
    startServer();
    auto second = collect<Joy>();
    ASSERT_TRUE(publishUntil<Joy>(
        publisher,
        joy(0.9f),
        [&]
        {
            return second->contains(joy(0.9f));
        },
        12s));
    EXPECT_FALSE(second->contains(joy(0.1f)));
}

TEST_F(BridgeIntegration, ComponentShutdownIsBoundedDuringRegistration)
{
    auto entered = std::make_shared<std::atomic<bool>>(false);
    auto management = publisherNode->create_service<Manage>(
        serverName + "/control_signal_manage",
        [entered](const Manage::Request::SharedPtr, Manage::Response::SharedPtr response)
        {
            entered->store(true);
            std::this_thread::sleep_for(900ms);
            response->response = Manage::Response::RESPONSE_SUCCESS;
        });
    startBridge(options().append_parameter_override("registration_timeout_ms", int64_t{300}));
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.5f),
                                  [&]
                                  {
                                      return entered->load();
                                  }));
    executor->remove_node(bridge);
    const auto start = std::chrono::steady_clock::now();
    bridge.reset();
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1200ms);
}

TEST_F(BridgeIntegration, ComponentShutdownIsBoundedDuringServiceSend)
{
    auto management =
        publisherNode->create_service<Manage>(serverName + "/control_signal_manage",
                                              [](const Manage::Request::SharedPtr, Manage::Response::SharedPtr response)
                                              {
                                                  response->response = Manage::Response::RESPONSE_SUCCESS;
                                              });
    auto entered = std::make_shared<std::atomic<bool>>(false);
    auto data = publisherNode->create_service<JoyService>(
        channel,
        [entered](const JoyService::Request::SharedPtr, JoyService::Response::SharedPtr response)
        {
            entered->store(true);
            std::this_thread::sleep_for(600ms);
            response->response = JoyService::Response::SRV_RES_SUCCESS;
        });
    startBridge(options("joy", "service"));
    auto publisher = publisherNode->create_publisher<Joy>(inputTopic, 10);
    ASSERT_TRUE(publishUntil<Joy>(publisher,
                                  joy(0.5f),
                                  [&]
                                  {
                                      return entered->load();
                                  }));
    executor->remove_node(bridge);
    const auto start = std::chrono::steady_clock::now();
    bridge.reset();
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1200ms);
}

TEST_F(BridgeIntegration, InvalidNodeParametersFailBeforeRegistering)
{
    for (const auto& invalid : {options().append_parameter_override("topic_name", std::string{}),
                                options().append_parameter_override("priority", int64_t{256}),
                                options().append_parameter_override("input_timeout_ms", int64_t{0}),
                                options().append_parameter_override("csm_mode", std::string{"invalid"}),
                                options("string", "service")})
        EXPECT_THROW(std::make_shared<Bridge>(invalid), std::invalid_argument);
}
}  // namespace
