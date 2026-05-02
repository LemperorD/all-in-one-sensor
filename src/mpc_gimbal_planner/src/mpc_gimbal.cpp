#include "mpc_gimbal_planner/mpc_gimbal.hpp"

namespace mpc_gimbal_planner
{
MPCGimbal::MPCGimbal()
{

}

MPCGimbal::~MPCGimbal()
{
}

} // namespace mpc_gimbal_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mpc_gimbal_planner::PlannerNode)