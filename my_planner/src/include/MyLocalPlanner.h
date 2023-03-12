#ifndef MY_LOCAL_PLANNER_H_
#define MY_LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mutex>


#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/tf.h>


using namespace std;

namespace my_local_planner
{
    typedef struct PositionDelta {
        double x;
        double y;        
        double angle;
    } PositionDelta;

    class MyLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        MyLocalPlanner();
        MyLocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        ~MyLocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        std::string myName = "MyLocalPlanner";

        

    private:
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        ros::Subscriber odom_sub_; 
        ros::Publisher vis_pub;
        costmap_2d::Costmap2DROS *myCostmapRos;
        costmap_2d::Costmap2D *myCostmap;
        tf2_ros::Buffer *tf_;
        bool initialized_;
        uint64_t markerId;
        PositionDelta lastPositionDelta;
        
        std::vector<geometry_msgs::PoseStamped> global_plan;

        base_local_planner::OdometryHelperRos odom_helper;
        base_local_planner::LocalPlannerUtil planner_util;
        geometry_msgs::PoseStamped current_pose_;
        std::string odom_topic_;
        std::mutex odom_mutex_; 
        nav_msgs::Odometry base_odom_;

        double distanceToWaitpointThreshold = 0.15;
        double distanceToVelocityScalar = 1.0;

        int currentGlobalPlanIndex = 0;

      

      
    };

}
#endif