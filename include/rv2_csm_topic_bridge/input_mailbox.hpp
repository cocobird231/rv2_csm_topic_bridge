#ifndef RV2_CSM_TOPIC_BRIDGE_INPUT_MAILBOX_HPP
#define RV2_CSM_TOPIC_BRIDGE_INPUT_MAILBOX_HPP

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <variant>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

namespace rv2_csm_topic_bridge
{
using InputMessage = std::variant<sensor_msgs::msg::Joy::ConstSharedPtr,
                                  geometry_msgs::msg::Twist::ConstSharedPtr,
                                  std_msgs::msg::String::ConstSharedPtr>;
using InputClock = std::chrono::steady_clock;
struct InputSample
{
    InputMessage message;
    InputClock::time_point receivedAt;
    uint64_t sequence;
    bool fresh(InputClock::time_point now, std::chrono::milliseconds maxAge) const
    {
        return now >= receivedAt && now - receivedAt <= maxAge;
    }
};

/** A bounded latest-value mailbox; callbacks never block on transport calls. */
class InputMailbox
{
public:
    void submit(InputMessage message, InputClock::time_point now = InputClock::now())
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_)
                return;
            sample_ = InputSample{std::move(message), now, ++sequence_};
        }
        wake_.notify_one();
    }
    std::optional<InputSample> wait(uint64_t& observed, std::chrono::milliseconds interval)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock,
                       interval,
                       [&]
                       {
                           return stopped_ || sequence_ != observed;
                       });
        observed = sequence_;
        return sample_;
    }
    std::optional<InputSample> latest() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sample_;
    }
    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
            sample_.reset();
        }
        wake_.notify_all();
    }
    bool stopped() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::optional<InputSample> sample_;
    uint64_t sequence_{0};
    bool stopped_{false};
};
}  // namespace rv2_csm_topic_bridge
#endif
