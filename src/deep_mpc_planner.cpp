#include <deep_mpc_local_planner/deep_mpc_planner.h>

namespace deep_mpc_local_planner
{
    DeepMPCPlanner::DeepMPCPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util)
    {
        ros::NodeHandle private_nh("~/" + name);
    }

    bool DeepMPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        return planner_util_->setPlan(orig_global_plan);
    }
}