#ifndef DEEP_MPC_PLANNER
#define DEEP_MPC_PLANNER

#include <Eigen/Core>

#include <base_local_planner/map_grid_visualizer.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

namespace deep_mpc_local_planner
{
    class DeepMPCPlanner
    {
    public:
        DeepMPCPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
        ~DeepMPCPlanner();
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped &global_pose,
                                     const std::vector<geometry_msgs::PoseStamped> &new_plan,
                                     const std::vector<geometry_msgs::Point> &footprint_spec);
        double getSimPeriod() { return sim_period_; }
        bool checkTrajectory(
            const Eigen::Vector3f pos,
            const Eigen::Vector3f vel,
            const Eigen::Vector3f vel_samples);
        base_local_planner::Trajectory findBestPath(
            const geometry_msgs::PoseStamped &global_pose,
            const geometry_msgs::PoseStamped &global_vel,
            geometry_msgs::PoseStamped &drive_velocities);

    private:
        Eigen::Vector3f vsamples_;
        base_local_planner::SimpleTrajectoryGenerator generator_;
        base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
        base_local_planner::OscillationCostFunction oscillation_costs_;
        base_local_planner::LocalPlannerUtil *planner_util_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        double sim_period_ = 0.05;
    };

}

#endif