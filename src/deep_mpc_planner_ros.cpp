#include <deep_mpc_local_planner/deep_mpc_planner_ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(deep_mpc_local_planner::DeepMPCPlannerROS, nav_core::BaseLocalPlanner);

namespace deep_mpc_local_planner
{
    DeepMPCPlannerROS::DeepMPCPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}
    DeepMPCPlannerROS::DeepMPCPlannerROS(std::string name, tf2_ros::Buffer *tf,
                                         costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }
    DeepMPCPlannerROS::~DeepMPCPlannerROS(){};

    void DeepMPCPlannerROS::initialize(
        std::string name,
        tf2_ros::Buffer *tf,
        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);

            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

            planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

            _robot_base_frame = costmap_ros_->getBaseFrameID();

            initialized_ = true;
        }
    }

    bool DeepMPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        ROS_ERROR("SET PLAN");
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // _global_plan is the curve from the robot to the target in rviz
        _global_plan.clear();
        _global_plan = orig_global_plan;
    }

    bool DeepMPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        ROS_ERROR("VEL COMMAND");
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
        {
            ROS_ERROR("Could not get local plan");
            return false;
        }

        if (transformed_plan.empty())
        {
            ROS_WARN_NAMED("deep_mpc_local_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("deep_mpc_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

        // ROS_ERROR("setting cmd_vel !!!!!!!!!!!!!!!!!!!!!!!!!1");
        // cmd_vel.linear.x = 5;

        return true;
    }

    bool DeepMPCPlannerROS::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
        {
            ROS_INFO("Goal reached");
            return true;
        }
        else
        {
            return false;
        }

        return false;
    }
}