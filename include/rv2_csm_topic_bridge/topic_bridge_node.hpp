#ifndef RV2_CSM_TOPIC_BRIDGE_TOPIC_BRIDGE_NODE_HPP
#define RV2_CSM_TOPIC_BRIDGE_TOPIC_BRIDGE_NODE_HPP
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace rv2_csm_topic_bridge
{
/** Forward live upstream input through typed R1 handles on a worker thread. */
class TopicBridgeNode : public rclcpp::Node
{
public:
    explicit TopicBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TopicBridgeNode() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace rv2_csm_topic_bridge
#endif
