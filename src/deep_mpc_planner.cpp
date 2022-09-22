#include <deep_mpc_local_planner/deep_mpc_planner.h>
#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>


namespace deep_mpc_local_planner
{
    DeepMPCPlanner::DeepMPCPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util)
    {
        ros::NodeHandle private_nh("~/" + name);
    }

    bool DeepMPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        oscillation_costs_.resetOscillationFlags();
        std::cout << "!!!!!!!!!!!11\n";
        return planner_util_->setPlan(orig_global_plan);
    }

    void DeepMPCPlanner::updatePlanAndLocalCosts(
        const geometry_msgs::PoseStamped &global_pose,
        const std::vector<geometry_msgs::PoseStamped> &new_plan,
        const std::vector<geometry_msgs::Point> &footprint_spec)
    {
        global_plan_.resize(new_plan.size());
        for (unsigned int i = 0; i < new_plan.size(); ++i)
        {
            global_plan_[i] = new_plan[i];
        }
    }
    bool DeepMPCPlanner::checkTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        Eigen::Vector3f vel_samples)
    {
        oscillation_costs_.resetOscillationFlags();
        base_local_planner::Trajectory traj;
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
        generator_.initialise(pos,
                              vel,
                              goal,
                              &limits,
                              vsamples_);
        generator_.generateTrajectory(pos, vel, vel_samples, traj);
        double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
        // if the trajectory is a legal one... the check passes
        if (cost >= 0)
        {
            return true;
        }
        ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

        // otherwise the check fails
        return false;
    }
}