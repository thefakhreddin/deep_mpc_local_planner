#include <deep_mpc_local_planner/deep_mpc_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

PLUGINLIB_EXPORT_CLASS(deep_mpc_local_planner::DeepMPCPlannerROS, nav_core::BaseLocalPlanner);

namespace deep_mpc_local_planner
{
    DeepMPCPlannerROS::DeepMPCPlannerROS() : initialized_(false),
                                             odom_helper_("odom"), setup_(false) {}

    // DeepMPCPlannerROS::DeepMPCPlannerROS(std::string name, tf2_ros::Buffer *tf,
    //                                      costmap_2d::Costmap2DROS *costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    // {
    //     initialize(name, tf, costmap_ros);
    // }

    // DeepMPCPlannerROS::~DeepMPCPlannerROS(){};

    void DeepMPCPlannerROS::initialize(
        std::string name,
        tf2_ros::Buffer *tf,
        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);

            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

            planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

            dp_ = boost::shared_ptr<DeepMPCPlanner>(new DeepMPCPlanner(name, &planner_util_));

            if (private_nh.getParam("odom_topic", odom_topic_))
            {
                odom_helper_.setOdomTopic(odom_topic_);
            }

            initialized_ = true;
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
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

        // latchedStopRotateController_.resetLatching();
        // return dp_->setPlan(orig_global_plan);

        // _global_plan is the curve from the robot to the target in rviz
        // _global_plan.clear();
        // _global_plan = orig_global_plan;
        ROS_ERROR("SET PLAN!!");
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

    void DeepMPCPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void DeepMPCPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }

    DeepMPCPlannerROS::~DeepMPCPlannerROS()
    {
    }

    bool DeepMPCPlannerROS::deepComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

        base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
        
        ROS_ERROR("deepComputeVelocityCommands");
        //..................................
        cmd_vel.linear.y = 0.5;
        cmd_vel.angular.z = 0.6;
        //................................

        std::vector<geometry_msgs::PoseStamped> local_plan;
        if (path.cost_ < 0)
        {
            ROS_DEBUG_NAMED("deep_mpc_local_planner",
                            "The deep mpc local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
            local_plan.clear();
            publishLocalPlan(local_plan);
            return false;
        }

        for (unsigned int i = 0; i < path.getPointsSize(); ++i)
        {
            double p_x, p_y, p_th;
            path.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            // tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }

        publishLocalPlan(local_plan);
        return true;
    }

    bool DeepMPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        ROS_ERROR("computeVelocityCommands");
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
            ROS_WARN_NAMED("deep_local_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("deep_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

        dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());
        if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
        {
            // publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            return latchedStopRotateController_.computeVelocityCommandsStopRotate(
                cmd_vel,
                limits.getAccLimits(),
                dp_->getSimPeriod(),
                &planner_util_,
                odom_helper_,
                current_pose_,
                boost::bind(&DeepMPCPlanner::checkTrajectory, dp_, _1, _2, _3));
        }
        else
        {
            bool isOk = deepComputeVelocityCommands(current_pose_, cmd_vel);
            if (isOk)
            {
                publishGlobalPlan(transformed_plan);
            }
            else
            {
                ROS_WARN_NAMED("deep_mpc_local_planner", "deep mpc planner failed to produce path.");
                std::vector<geometry_msgs::PoseStamped> empty_plan;
                publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
    }
}
