#ifndef DEEP_MPC_PLANNER
#define DEEP_MPC_PLANNER

#include <base_local_planner/local_planner_util.h>

namespace deep_mpc_local_planner
{
    class DeepMPCPlanner
    {
    public:
        DeepMPCPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
        ~DeepMPCPlanner();
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

    private:
        base_local_planner::LocalPlannerUtil *planner_util_;
    };

}

#endif