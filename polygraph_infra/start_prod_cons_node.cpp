/*!
 * \file start_system_node.cpp
 * \brief Source file for the start system node (contact A. Berne if errors)
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 1.0
 * \date 05/07/2023
 *
 * This file is dedicated to the start system node (triggering all clocked nodes)
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

/*!
 * \fn int main (int argc, char **argv)
 * \brief Entry point of the start_system_node program
 *
 * \param[in] argc Containing number of arguments passed in the command line
 * \param[in] argv Containing arguments passed in the command line
 *
 * \return EXIT_SUCCESS - Program successfully executed.
 */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "start_prod_cons_node");
  ros::NodeHandle nh;

  // Creating start system publisher
  ros::Publisher start_pub = nh.advertise<std_msgs::String>("/sys_trigger", 1);

  // Preparing start system message
  std_msgs::String msg;
  std::stringstream ss;
  ss << "prod_cons started !";
  msg.data = ss.str();

  // Loop every 1s
  int loop = 5;
  ros::Rate rate(1.);
  printf("Starting prod_cons project\n");
  while(ros::ok() && loop > 0)
  {
    printf("%d... ", loop);
    fflush(stdout);
    loop--;

    // ROS sleep
    rate.sleep();
  }
  printf("\n");
  start_pub.publish(msg);
  ros::spinOnce();
  // ROS sleep before quitting program
  rate.sleep();

  return 0;
}