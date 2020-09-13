#include <ros/ros.h>

#include <erc_supervisor/ErcSupervisor.hpp>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "erc_supervisor");

    erc::ErcSupervisor supervisor;
    
    ros::Rate r(10);

    while(ros::ok())
    {
        ros::spinOnce();
        supervisor.Update();
        r.sleep();
    }

    return 0;
}