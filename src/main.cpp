#include <iostream>
#include "base_server/xmbaseserver.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"base_server");
    ros::NodeHandle n;
    XmBaseServer  server(n);
    ros::AsyncSpinner spiner(4);
    spiner.start();
    server.main_run();

    ros::waitForShutdown();
    return 0;
}

