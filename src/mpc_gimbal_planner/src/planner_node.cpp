#include "mpc_gimbal_planner/planner_node.hpp"

namespace mpc_gimbal_planner
{
PlannerNode::PlannerNode(const rclcpp::NodeOptions & options)
: Node("planner_node", options)
{

}

PlannerNode::~PlannerNode()
{
}

void PlannerNode::onConfigure()
{

}

} // namespace mpc_gimbal_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mpc_gimbal_planner::PlannerNode)