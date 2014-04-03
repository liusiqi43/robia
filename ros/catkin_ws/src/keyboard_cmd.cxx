
#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_cmd");

    ros::NodeHandle n;
    std_msgs::Empty emptyMsg;

    // keep queue size to 1
    ros::Publisher pubTakeOff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pubLand = n.advertise<std_msgs::Empty>("/ardrone/land", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        char c = getchar();

        switch(c) {
            case 't' : {
                           pubTakeOff.publish(emptyMsg);
                           ROS_INFO("taking off...");
                           break;
                       }
            case 'l' : {
                           pubLand.publish(emptyMsg);
                           ROS_INFO("landing...");
                           break;
                       }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
