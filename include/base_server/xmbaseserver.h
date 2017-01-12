#ifndef XMBASESERVER_H
#define XMBASESERVER_H
#include <iostream>
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Pose2D.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>

#include<std_msgs/Bool.h>
#include<std_msgs/String.h>

#include<nav_msgs/Odometry.h>
#include<ros/service_server.h>
#include<std_srvs/Trigger.h>

//#include<actionlib/client/simple_action_client.h>
//#include<move_base_msgs/MoveBaseAction.h>
#include<xm_msgs/xm_Move.h>

#include<people_msgs/PositionMeasurementArray.h>
#include<xm_msgs/xm_People.h>

#include<boost/shared_ptr.hpp>
#include<boost/make_shared.hpp>


//follow_planner
#include<costmap_2d/costmap_2d_ros.h>
#include<tf/transform_listener.h>
#include<dwa_local_planner/dwa_planner_ros.h>


//typedef boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > MoveBaseClientPtr;

#include"base_server/follower.h"

class XmBaseServer
{
public:
    XmBaseServer(ros::NodeHandle n);
    void laser_callback(const sensor_msgs::LaserScanConstPtr &msg);
    void odom_callback(const nav_msgs::OdometryConstPtr &msg);
    void cmd_callback(const std_msgs::StringConstPtr &msg);
    bool move_callback(xm_msgs::xm_Move::Request &req,xm_msgs::xm_Move::Response &res);

    void people_callback(const xm_msgs::xm_PeopleConstPtr &msg);
    void laser_people_callback(const people_msgs::PositionMeasurementArrayConstPtr &msg);
    bool service_callback(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);
    void leg_callback(const people_msgs::PositionMeasurementArrayConstPtr &msg);
    void main_run();
private:
    inline double get_dis(double x, double y)
    {
        return sqrt(x*x + y*y);
    }

    void pub_door_state(const sensor_msgs::LaserScanConstPtr &msg);
    void scan_filter(const sensor_msgs::LaserScanConstPtr &msg);
    void scan_filter(const sensor_msgs::LaserScanConstPtr &scan , const people_msgs::PositionMeasurementArray &leg_position);
    ros::NodeHandle nh;

    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;
    ros::Publisher  door_pub;
    ros::Subscriber people_sub;
    ros::Subscriber laser_people_sub;


    ros::Publisher  vel_pub;
    ros::Publisher  laser_pub;
    ros::Publisher  laser_slam_pub;
    ros::ServiceServer server;
    ros::ServiceServer move_server;
   // ros::Publisher goal_pub;
    //MoveBaseClientPtr client_ptr;

    geometry_msgs::PoseStamped people_position;

    xm_msgs::xm_People camera_people;
    int people_id;
    std::string people_name;
    people_msgs::PositionMeasurementArray people_arry;

    std::string way_of_follow; // laser or camera
    ros::Time  temp_time;
    bool lost_flag;


    geometry_msgs::Pose init_pose;
    geometry_msgs::Pose   current_pose;
    geometry_msgs::Pose  goal_pose;

    bool follow_state;
    bool door_state; // open 1 close 0

    bool detect_door;
    bool shopping;


};

#endif // XMBASESERVER_H

