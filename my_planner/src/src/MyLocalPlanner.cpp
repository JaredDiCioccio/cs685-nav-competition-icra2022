#include "MyLocalPlanner.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

PLUGINLIB_EXPORT_CLASS(my_local_planner::MyLocalPlanner, nav_core::BaseLocalPlanner)

namespace my_local_planner
{

    MyLocalPlanner::MyLocalPlanner() : myCostmapRos(NULL), tf_(NULL), initialized_(false) {}

    MyLocalPlanner::MyLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                                   costmap_2d::Costmap2DROS *costmap_ros_)
        : myCostmapRos(NULL), tf_(NULL), initialized_(false), odom_helper("odom")
    {
        initialize(name, tf, costmap_ros_);
    }

    MyLocalPlanner::~MyLocalPlanner() {}

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void MyLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                    costmap_2d::Costmap2DROS *costmap_ros_)
    {
        if (!initialized_)
        {
            ROS_INFO_NAMED(myName, "Initializing MyLocalPlanner.cpp [BEGIN]");
            tf_ = tf;
            myCostmapRos = costmap_ros_;
            myCostmap = myCostmapRos->getCostmap();

            // myCostmapRos->getRobotPose(current_pose_);
            planner_util.initialize(tf, myCostmap, myCostmapRos->getGlobalFrameID());

            ros::NodeHandle gn;
            odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, &MyLocalPlanner::odomCallback, this);

            initialized_ = true;
            ROS_INFO_NAMED(myName, "Initializing MyLocalPlanner.cpp [DONE]");
        }
        else
        {
            ROS_WARN_NAMED(myName, "Planner already initialized.");
        }
    }

    bool MyLocalPlanner::setPlan(
        const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        bool success = planner_util.setPlan(orig_global_plan);
        if (global_plan.size() != orig_global_plan.size())
        {
            global_plan = orig_global_plan;
            currentGlobalPlanIndex = 1;

            ROS_INFO_COND_NAMED(success, myName, "Set new global plan");
            ROS_INFO_NAMED(myName, "My global plan size = %ld", global_plan.size());
            ROS_WARN_COND_NAMED(!success, myName, "Failed to set new global plan");
        }

        return success;
    }

    bool MyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        geometry_msgs::PoseStamped target;
        geometry_msgs::PoseStamped curr_pose;
        ROS_INFO_THROTTLE_NAMED(1, myName, "global_plan.size()= %ld", global_plan.size());

        if (global_plan.size() == 0)
        {
            ROS_WARN_NAMED(myName, "Global Plan has no entries");
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;

            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0;
            return true;
        }

        // for(std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin(); it != global_plan.end(); ++it){
        //     geometry_msgs::PoseStamped pose = *it;
        //     ROS_INFO("%f, %f, %f", pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        // }
        ROS_INFO_THROTTLE_NAMED(0.5, myName, "Getting current pose from costmap");
        if (!myCostmapRos->getRobotPose(current_pose_))
        {
            ROS_ERROR_NAMED(myName, "Could not get robot pose");
        }

        // Calculate error between where we are and next step of global plan
        ROS_INFO_THROTTLE_NAMED(0.5, myName, "Calculating position delta based on waypoint #%d", currentGlobalPlanIndex);
        PositionDelta delta;

        geometry_msgs::PoseStamped nextWaypoint = global_plan[currentGlobalPlanIndex];

        delta.x = nextWaypoint.pose.position.x - current_pose_.pose.position.x;
        delta.y = nextWaypoint.pose.position.y - current_pose_.pose.position.y;
        double deltaZ = nextWaypoint.pose.position.z - current_pose_.pose.position.z;
        ROS_INFO_THROTTLE_NAMED(0.5, myName, "Position delta = [%.2f, %.2f]", delta.x, delta.y);
        ROS_WARN_COND_NAMED(deltaZ > 0.10, myName, "Uh oh I think we're flying");

        // Calculate smallest angle between positions
        // Angle between points is
        // Thanks https://math.stackexchange.com/questions/707673/find-angle-in-degrees-from-one-point-to-another-in-2d-space
        // (I'm not great at geometry)
        double angleRadians;
        if (delta.x != 0 && delta.y != 0)
        {
            // see https://en.wikipedia.org/wiki/Atan2#Argument_order
            angleRadians = std::atan2(delta.y, delta.x); // radians
        }
        else
        {
            // atan2 of 0,0 is undefined
            angleRadians = lastPositionDelta.angle; // or 0?
        }

        

        tf::Quaternion quat(
            curr_pose.pose.orientation.x,
            curr_pose.pose.orientation.y,
            curr_pose.pose.orientation.z,
            curr_pose.pose.orientation.w);
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // convert radians to degrees so it's easier for squishy human brains to comprehend
        delta.angle = angleRadians * (180 / M_PI);

        if (delta.angle > 180)
        {
            delta.angle -= 360;
        }
        if (delta.angle < -180)
        {
            delta.angle += 360;
        }

        double distanceToGoal0 = std::sqrt(std::pow(delta.x, 2) + std::pow(delta.y, 2));

        double projectedX = 0.75 * cos(angleRadians);
        double projectedY = 0.75 * sin(angleRadians);

        // see if we're too close to an object

        if (distanceToGoal0 > distanceToWaitpointThreshold)
        {
            if (fabs(delta.angle) < 15)
            {
                // If we're generally pointing in the right direction, set a small angular velocity
                cmd_vel.linear.x = distanceToGoal0 * distanceToVelocityScalar;
                if (myCostmap->getCost(projectedX, projectedY) < 0)
                {
                    // if there's an obstacle 2.5 meters in front of us
                    cmd_vel.linear.x = -1;
                }
                cmd_vel.angular.z = 0.15 * (delta.angle > 0) - (delta.angle < 0);
                return true;
            }
            else
            {
                // If we're way off, slow down to try to point in the right direction
                cmd_vel.angular.z = .30 * (delta.angle > 0) - (delta.angle < 0);
                cmd_vel.linear.x = 0.1;
                return true;
            }
        }
        else
        {
            ROS_INFO_NAMED(myName, "Looks like we reached a waypoint");
            cmd_vel.linear.x = 0.01;
            cmd_vel.angular.z = 0;
            currentGlobalPlanIndex += 1;
            return true;
        }

        cmd_vel.linear.y = cmd_vel.linear.y;
        cmd_vel.linear.z = cmd_vel.linear.z;
        lastPositionDelta = delta;
        return true;
    }

    bool MyLocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        bool goalReached = false;

        if (goalReached)
        {
            ROS_INFO("Goal reached");
            return true;
        }
        else
        {
            return false;
        }
    }

    void MyLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // lock Callback while reading data from topic
        std::scoped_lock lock(odom_mutex_);

        // get odometry and write it to member variable (we assume that the odometry is published in the frame of the base)
        base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
        base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
        base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
        // ROS_INFO_NAMED(myName, "odomCallback: %f, %f, %f", base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
    }
}