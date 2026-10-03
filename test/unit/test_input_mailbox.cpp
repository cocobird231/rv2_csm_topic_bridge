#include <future>
#include <gtest/gtest.h>
#include "rv2_csm_topic_bridge/input_mailbox.hpp"

using namespace std::chrono_literals;
using namespace rv2_csm_topic_bridge;
using Joy = sensor_msgs::msg::Joy;

TEST(InputMailbox, LatestSampleReplacesBlockedInputWithoutUnboundedQueue)
{
    InputMailbox mailbox;
    EXPECT_FALSE(mailbox.latest());
    const auto now = InputClock::now();
    auto first = std::make_shared<Joy>();
    first->axes = {0.25f};
    auto second = std::make_shared<Joy>();
    second->axes = {-0.75f};
    mailbox.submit(Joy::ConstSharedPtr(first), now);
    mailbox.submit(Joy::ConstSharedPtr(second), now + 1ms);
    const auto latest = mailbox.latest();
    ASSERT_TRUE(latest);
    EXPECT_EQ(latest->sequence, 2u);
    EXPECT_EQ(latest->receivedAt, now + 1ms);
    EXPECT_EQ(std::get<Joy::ConstSharedPtr>(latest->message)->axes, second->axes);
    uint64_t observed = 0;
    ASSERT_TRUE(mailbox.wait(observed, 0ms));
    EXPECT_EQ(observed, 2u);
}

TEST(InputMailbox, FreshnessUsesReceiptClockWithExactDeadline)
{
    const auto now = InputClock::now();
    InputSample sample{Joy::ConstSharedPtr(std::make_shared<Joy>()), now, 1};
    EXPECT_FALSE(sample.fresh(now - 1ns, 100ms));
    EXPECT_TRUE(sample.fresh(now, 100ms));
    EXPECT_TRUE(sample.fresh(now + 100ms, 100ms));
    EXPECT_FALSE(sample.fresh(now + 100ms + 1ns, 100ms));
}

TEST(InputMailbox, SubmissionWakesWaitingWorker)
{
    InputMailbox mailbox;
    auto waiter = std::async(std::launch::async,
                             [&]
                             {
                                 uint64_t observed = 0;
                                 return mailbox.wait(observed, 1s);
                             });
    mailbox.submit(Joy::ConstSharedPtr(std::make_shared<Joy>()));
    ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
    ASSERT_TRUE(waiter.get());
}

TEST(InputMailbox, StopWakesWorkerClearsSampleAndRejectsLaterCallbacks)
{
    InputMailbox mailbox;
    mailbox.submit(Joy::ConstSharedPtr(std::make_shared<Joy>()));
    auto waiter = std::async(std::launch::async,
                             [&]
                             {
                                 uint64_t observed = 1;
                                 return mailbox.wait(observed, 1s);
                             });
    mailbox.stop();
    mailbox.stop();
    EXPECT_TRUE(mailbox.stopped());
    ASSERT_EQ(waiter.wait_for(500ms), std::future_status::ready);
    EXPECT_FALSE(waiter.get());
    mailbox.submit(Joy::ConstSharedPtr(std::make_shared<Joy>()));
    EXPECT_FALSE(mailbox.latest());
}
