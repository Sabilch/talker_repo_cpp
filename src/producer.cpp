#include "ros/ros.h"
#include <std_msgs/String.h>

int main (int args, char **argv) {
    ros::init(args, argv, "producer");
    ros::NodeHandle nh;

    ros::Publisher topic_prod = nh.advertise<std_msgs::String>("/Data",1);

    ros::Rate loop_rate(5);
    
    while(ros::ok()) {
        std_msgs::String msg;

        msg.data = "here's the publisher";

        topic_prod.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
