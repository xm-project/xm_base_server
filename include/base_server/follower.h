#ifndef FOLLOWER_H
#define FOLLOWER_H
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>



class follower
{
public:
    follower(ros::NodeHandle &n);
    void update_position(geometry_msgs::Point position);
    void compute_velocity(geometry_msgs::Twist &twist);
private:
    void laser_callback(const sensor_msgs::LaserScanConstPtr &msg);
    geometry_msgs::Point pos;
    ros::Time last_update_time;
    ros::NodeHandle nh;
    ros::Subscriber laser_sub;
    std::vector<int> observation;// 0 1 2 3   left  left_front right_front right
    double max_x_vel;
    double max_y_vel;
    double max_angual_vel;
    double max_dis;
    double range_dis;
    double belief;

};

#endif // FOLLOWER_H
