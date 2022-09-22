#ifndef DEEP_MPC_PLANNER_ROS
#define DEEP_MPC_PLANNER_ROS

#include <boost/shared_ptr.hpp>
#include <tf2_ros/buffer.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <deep_mpc_local_planner/deep_mpc_planner.h>

namespace deep_mpc_local_planner
{
    class DeepMPCPlannerROS : public nav_core::BaseLocalPlanner
    {
    public:
        DeepMPCPlannerROS();
        DeepMPCPlannerROS(std::string name, tf2_ros::Buffer *tf,
                          costmap_2d::Costmap2DROS *costmap_ros);
        ~DeepMPCPlannerROS();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
        bool deepComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        bool isGoalReached();

    private:
        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);
        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);
        bool setup_;
        ros::Publisher g_plan_pub_, l_plan_pub_;
        tf2_ros::Buffer *tf_;
        costmap_2d::Costmap2DROS *costmap_ros;
        bool initialized_;
        std::string odom_topic_;
        geometry_msgs::PoseStamped current_pose_;
        costmap_2d::Costmap2DROS *costmap_ros_;
        base_local_planner::LocalPlannerUtil planner_util_;
        base_local_planner::LatchedStopRotateController latchedStopRotateController_;
        base_local_planner::OdometryHelperRos odom_helper_;
        std::string _robot_base_frame;
        boost::shared_ptr<DeepMPCPlanner> dp_;
        std::vector<geometry_msgs::PoseStamped> _global_plan;
    };

}

#endif