#include "base_server/xmbaseserver.h"
#include <tf/tf.h>
#include <cmath>
#include <algorithm>
#include <unistd.h>

XmBaseServer::XmBaseServer(ros::NodeHandle n)
    :nh(n)

{
    people_id = 999;
    door_state = false;
    follow_state = false;
    lost_flag = false;
    nh.param("detect_door",detect_door,true);
    nh.param("shopping",shopping,false);
    nh.param("way_of_follow",way_of_follow,std::string("laser"));

    laser_sub = nh.subscribe("/scan",100,&XmBaseServer::laser_callback,this);
    odom_sub  = nh.subscribe("/xm_robot/mobile_base_controller/odom",100,&XmBaseServer::odom_callback,this);

    server   = nh.advertiseService("follow",&XmBaseServer::service_callback,this);
    move_server =nh.advertiseService("move",&XmBaseServer::move_callback,this);
    laser_people_sub = nh.subscribe("/people_tracker_measurements",1000,&XmBaseServer::laser_people_callback,this);
    people_sub= nh.subscribe("/People",20,&XmBaseServer::people_callback,this);

    //goal_pub =nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    vel_pub   = nh.advertise<geometry_msgs::Twist>("/xm_robot/mobile_base_controller/cmd_vel",100);
    door_pub  = nh.advertise<std_msgs::Bool>("/DoorState",1);
    laser_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_filtered",10);
    laser_slam_pub = nh.advertise<sensor_msgs::LaserScan>("/slam_scan",10);

}


bool XmBaseServer::service_callback(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
{
    if(follow_state == false){
    follow_state = true;
    res.success = true;
    res.message = "start";
    temp_time = ros::Time::now();
    }

    else{
        follow_state = false;
        res.success = true;
        res.message = "stop";
    }
    return true;

}

void XmBaseServer::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
    current_pose = (msg->pose).pose;
    double yaw =tf::getYaw((msg->pose).pose.orientation);
    //ROS_INFO("New position  x : %f  y : %f  theta: %f",current_pose.position.x,current_pose.position.y,yaw);
}


void XmBaseServer::people_callback(const xm_msgs::xm_PeopleConstPtr &msg)
{
    ROS_INFO("New Openni people position come! ");
    camera_people = *msg;
}

void XmBaseServer::laser_people_callback(const people_msgs::PositionMeasurementArrayConstPtr &msg)
{

    people_arry = *msg;
    ROS_INFO("Receive New People Position Arry !!!!!!!!!!!!!!!!!%d",msg->people.size());
    if (msg->people.size() ==0)
    {
    	if((ros::Time::now()-temp_time) > ros::Duration(1.0)) lost_flag = true;
    }
    /*
    else if(msg->people.size() ==1)
    {
    	temp_time = ros::Time::now();
    	lost_flag = false;	
    	people_name = msg->people[0].name;
    	people_position.pose.position.x = msg->people[0].pos.x;
        	people_position.pose.position.y = msg->people[0].pos.y;
        	people_position.pose.orientation = tf::createQuaternionMsgFromYaw (atan2( msg->people[0].pos.y,msg->people[0].pos.x) );
    }*/
    else 
    {
        temp_time = ros::Time::now();
        lost_flag = false;
        double temp_dis = 200 ;
        int temp;
        for(size_t i = 0 ; i < msg->people.size() ; i++ )
        {
            double dis;
            dis = get_dis(msg->people[i].pos.x , msg->people[i].pos.y);
            if(dis < temp_dis)
            {
                temp = i;
                temp_dis =dis;
            }
            if(msg->people[i].name==people_name)
            {
            	temp = i;
            	break;
            }
        }
        people_position.pose.position.x = msg->people[temp].pos.x;
        people_position.pose.position.y = msg->people[temp].pos.y;
        people_position.pose.orientation = tf::createQuaternionMsgFromYaw (atan2( msg->people[temp].pos.y,msg->people[temp].pos.x) );
    }

}

void XmBaseServer::scan_filter(const sensor_msgs::LaserScanConstPtr &msg)
{
     sensor_msgs::LaserScan filtered_scan;
     filtered_scan.ranges.resize(msg->ranges.size());
     unsigned int n = msg->ranges.size();
             //loop through the scan and reassign range values

     //make sure to set all the needed fields on the filtered scan
     filtered_scan.header.frame_id = msg->header.frame_id;
     filtered_scan.header.stamp = msg->header.stamp;
     filtered_scan.angle_min = msg->angle_min;
     filtered_scan.angle_max = msg->angle_max;
     filtered_scan.angle_increment = msg->angle_increment;
     filtered_scan.time_increment = msg->time_increment;
     filtered_scan.scan_time = msg->scan_time;
     filtered_scan.range_min = msg->range_min;
     filtered_scan.range_max = msg->range_max;
     for (unsigned int i = 0; i < n; ++i) filtered_scan.ranges[i] = msg->ranges[i];

     if(1)
    {
         if(people_arry.people.size()>0)
         {
             double dt_theta;
             double theta;
             for(size_t i = 0 ; i<people_arry.people.size() ; i++)
             {

                 theta = atan2(people_arry.people[i].pos.y,people_arry.people[i].pos.x);
                 dt_theta = atan( 0.2/get_dis(people_arry.people[i].pos.x,people_arry.people[i].pos.y));
                 int k = int( (theta - filtered_scan.angle_min)/filtered_scan.angle_increment);
                 int temp = int(dt_theta / filtered_scan.angle_increment);
                 int max,min;
                 max = std::min( int(n), k +temp);
                 min = std::max(0,k - temp);
                 for(size_t j = min ; j < max ;j++)
                 {
                     if(filtered_scan.ranges[j] < 3.0) filtered_scan.ranges[j] = NAN;
                 }


              }

        }

         laser_slam_pub.publish(filtered_scan);

     }



     for (unsigned int i = 0; i < n; ++i)
     {
         if(msg->ranges[i] > 5.0 ||isnan(msg->ranges[i]) ||isinf(msg->ranges[i]))
         {
             filtered_scan.ranges[i] = 5.0;
         }
         else
         {
             filtered_scan.ranges[i] = msg->ranges[i];
         }
     }

     laser_pub.publish(filtered_scan);

}


void XmBaseServer::pub_door_state(const sensor_msgs::LaserScanConstPtr &msg)
{
    double scan_x = 0.0;
    double scan_y = 0.0;
    int count = 0;
    std_msgs::Bool message;
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i*msg->angle_increment;
        scan_x = range * cos(angle);
        scan_y = range * sin(angle);
        if ( (scan_y < 0.22 || scan_y > - 0.22)&& scan_x >1.5 && scan_y < 4.0 && scan_y >-4.0)
        {
            count++;
        }
    }
    //ROS_INFO("%d",count);
    if (count > 40)
    {
        door_state = 1;
      //  ROS_INFO("the door is open:%d",door_state);
    }
    else{
        door_state = 0;
    }
        message.data = door_state;
        door_pub.publish(message);

}


void XmBaseServer::laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{

    scan_filter(msg);
    if(detect_door) pub_door_state(msg);

}

bool XmBaseServer::move_callback(xm_msgs::xm_Move::Request &req,xm_msgs::xm_Move::Response &res)
{
    bool goal_reached = false;
    init_pose = current_pose;
    ros::Rate r(30);
    int num =0;
    ROS_WARN("Move Cmd  %f , %f ,%f",req.x,req.y,req.yaw);
    if(fabs(req.x) > 0)
    {
        num = 50;
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.15 *(req.x > 0 ? 1.0 : -1.0);
        while(!goal_reached  && ros::ok() )
        {
            vel_pub.publish(cmd) ;
            cmd.angular.z = (tf::getYaw(init_pose.orientation) - tf::getYaw(current_pose.orientation))*1.0;
            double dis=sqrt((init_pose.position.x-current_pose.position.x)*(init_pose.position.x-current_pose.position.x)+ (init_pose.position.y-current_pose.position.y)* (init_pose.position.y-current_pose.position.y)); 
            std::cout<<"DIS"<<dis<<std::endl;
            if(dis >fabs(req.x) ) goal_reached =true;
            r.sleep();
        }
        res.arrived = true;
        return true;
    }

    if(fabs(req.y) > 0 )
    {
        geometry_msgs::Twist cmd;
        cmd.linear.y = 0.15*(req.y > 0 ? 1.0 : -1.0);
        num = 20;
        while(!goal_reached  && ros::ok() )
        {
            if(num >0)
            {
              cmd.angular.z = 0.15*(req.y > 0 ? 1.0 : -1.0);
              num--;
            }
            else{
                cmd.angular.z = (tf::getYaw(init_pose.orientation) - tf::getYaw(current_pose.orientation))*2.0;
            }
            vel_pub.publish(cmd);
            double dis=sqrt((init_pose.position.x-current_pose.position.x)*(init_pose.position.x-current_pose.position.x)+ (init_pose.position.y-current_pose.position.y)* (init_pose.position.y-current_pose.position.y)); 
            std::cout<<"DIS"<<dis<<std::endl;
            if(dis >fabs(req.y) ) goal_reached =true;
            r.sleep();
        }
        res.arrived =true;
        return true;
    }
    if(fabs(req.yaw) > 0)
    {
        geometry_msgs::Twist cmd;
        cmd.angular.z = 0.3*(req.yaw > 0 ? 1.0 : -1.0);
        while(!goal_reached  && ros::ok() )
        {
            vel_pub.publish(cmd);
            if( fabs(tf::getYaw(init_pose.orientation) - tf::getYaw(current_pose.orientation) )  > fabs(req.yaw) ) goal_reached =true;
            std::cout<<fabs(tf::getYaw(init_pose.orientation) - tf::getYaw(current_pose.orientation) )<<std::endl;
            r.sleep();
        }
         res.arrived =true;
         return true;
    }
    return true;
    }

void XmBaseServer::main_run()
{
    if(way_of_follow == "camera")
        {
                laser_people_sub.shutdown();
                if(follow_state == true)
                {
                    while(ros::ok())
                    {
                             if(follow_state == true)
                             {
                                 follower follow(nh);
                                 ros::Rate r(10);
                                 while(follow_state)
                                    {
                                        geometry_msgs::Point pos;
                                        bool find =false;
                                        if(camera_people.person.size() > 0)
                                           {
			     int i;
                                                if(people_id == 999)
                                                {
                                                    people_id = camera_people.person[0].id;
                                                }
                                                for(i =0 ;i<camera_people.person.size() ;i++)
                                                {
                                                        if(people_id == camera_people.person[i].id)
                                                        {
                                                            break;
                                                            find = true;
                                                        }
                                                }
                                                if(find)
                                                {
                                                    pos = camera_people.person[i].posi.point;
                                                    follow.update_position(pos);
                                                    geometry_msgs::Twist cmd;
                                                    follow.compute_velocity(cmd);
                                                    vel_pub.publish(cmd);
                                                    r.sleep();
                                                }
                                            }
                                       }
                                   }
                            }
                    }
       }
        if(way_of_follow == "laser")
        {
            people_sub.shutdown();
            while(ros::ok())
            {
             if(follow_state == true)
             {
                    follower follow(nh);
                    ros::Rate r(10);
                    while(follow_state)
                        {
                            geometry_msgs::Point pos;
                            geometry_msgs::Twist cmd;
                            if(!lost_flag)
                            {
                           		pos.x = people_position.pose.position.x;
                            	pos.y = people_position.pose.position.y;
                            	follow.update_position(pos);
                            	follow.compute_velocity(cmd);
                            }
                             vel_pub.publish(cmd);
                            r.sleep();
                       }
                   }
            }
        }
       /* if(follow_state == true){

            tf::TransformListener tf(ros::Duration(5));
            costmap_2d::Costmap2DROS costmap("follow_costmap",tf);
            costmap.start();
            dwa_local_planner::DWAPlannerROS planner;
            planner.initialize("follow_planner",&tf,&costmap);
            geometry_msgs::Twist twist;
            ros::Rate r(100);
            int count ;
            std::vector<geometry_msgs::PoseStamped> plan;
            ros::NodeHandle n;

            while(n.ok())
            {
                plan.clear();
                geometry_msgs::PoseStamped temp;
                temp.pose = current_pose;
                temp.header.frame_id = "base_link";
                temp.header.stamp = ros::Time::now();
                plan.push_back(temp);
                temp.pose = people_position.pose;
        temp.pose.position.x -=0.2;
                plan.push_back(temp);
                planner.setPlan(plan);
                planner.computeVelocityCommands(twist);
                ROS_WARN("compute command %f ,%f %f",twist.linear.x,twist.linear.y,twist.angular.z);

                   vel_pub.publish(twist);
                   r.sleep();
                }
           }*/
            //client_ptr = boost::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> >("move_base",true);
            //while(!client_ptr->waitForServer(ros::Duration(5.0)) &&(follow_state==true)&&ros::ok() ){
            //    ROS_INFO("Wait for server");
            // }
           //ROS_INFO("Connect to MoveBase successfully!! \n I will start Follow !!!");
            //ros::Rate rate(1);
          /*  while(follow_state)
            {
                ROS_INFO("Send Goal !! x:%f y:%f",people_position.pose.position.x,people_position.pose.position.y);
                //move_base_msgs::MoveBaseGoal goal;
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id = "base_link";
                goal.header.stamp = ros::Time::now();
                goal.pose = people_position.pose;
                goal_pub.publish(goal);
                rate.sleep();

                client_ptr->waitForResult();
                if(client_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("succeed");
                }
                else if(client_ptr->getState() ==actionlib::SimpleClientGoalState::PREEMPTED){
                    continue;
                }
                else if(client_ptr->getState() == actionlib::SimpleClientGoalState::ABORTED)
                {
                    client_ptr->sendGoal(goal);
                }
                rate.sleep();

            }*/
}


