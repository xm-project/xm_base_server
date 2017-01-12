#include "base_server/follower.h"

follower::follower(ros::NodeHandle &n):
    nh(n)
{
    laser_sub = nh.subscribe("/scan",10,&follower::laser_callback,this);
    max_x_vel = 1.2;
    max_y_vel = 0.5;
    max_angual_vel = 1.0;
    max_dis = 1.0;
    range_dis = 0.3;
    observation.resize(4,false);
    belief = 0.1;
}

void follower::update_position(geometry_msgs::Point position)
{
    pos = position;
    if(last_update_time - ros::Time::now() > ros::Duration(3)) ROS_WARN("position update time out!!!!!");
    last_update_time = ros::Time::now();

}

void follower::compute_velocity(geometry_msgs::Twist &twist)
{
    if(pos.x <max_dis)
    {
            twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	    twist.angular.z = pos.y*max_angual_vel;
    }
    else{
        twist.linear.x = std::min((pos.x - max_dis)*max_x_vel,0.3);
        
    //angular_speed
    twist.angular.z = pos.y*max_angual_vel;


    //way1
    if(observation[0])
    {
        //TODO change velocity use the belief
        twist.linear.y = -0.15;
    }
    if(observation[1])
    {
        twist.linear.y += -0.1;
    }
    if(observation[2])
    {
        twist.linear.y += 0.15;
    }
    if(observation[3])
    {
        twist.linear.y += 0.1;
    }
     }
}


void follower::laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    std::vector<int> count(4,0);
    int num = msg->ranges.size();

    for(size_t i =0 ; i< num/8 ; i++)
    {
        if(msg->ranges[i] < range_dis ) count[3]++;
    }
    for(size_t i = num/4 ; i < num*3/8 ; i++ )
    {
        if(msg->ranges[i] < range_dis+0.2 ) count[2]++;
    }
    for(size_t i = num*5/8 ; i<num*3/4 ; i++)
    {
        if(msg->ranges[i] <range_dis+0.2 ) count[1]++;
    }
    for(size_t i = num*7/8 ; i<num;i++ )
    {
        if(msg->ranges[i] < range_dis) count[0]++;
    }

    for(size_t i =0 ;i<4 ;i++)
    {
      observation[i] = count[i]/(double(num/8)) > belief ? 1 : 0;
    }
    ROS_INFO("state  %d ,%d , %d ,%d ",observation[0],observation[1],observation[2],observation[3]);

}

