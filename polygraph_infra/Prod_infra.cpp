/*!
 * \file Prod_infra.cpp
 * \brief Polygraph infrastructure source file for the Prod node
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 2.0
 * \date 05/07/2023
 *
 * Polygraph infrastructure source file for the Prod node - generated automatically (contact A. Berne if errors)
 * This file is dedicated to the Polygraph Infrastructure code
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 */

#include "Prod.h"
#include "polygraph_trace.h"
/******************************************************/
//             USER CODE CALLBACKS                    //
/******************************************************/
extern void Prod_setup (DataStructure *datastruct);
extern void Prod_loop (DataStructure *datastruct);

/******************************************************/
//              POLYGRAPH INFRA CODE                  //
/******************************************************/
// Topic Publisher declarations
ros::Publisher Prod_out_pub;

/*!
 * \struct TokenStructure
 * \brief Structure containing token values for each topics of the Prod node 
 *
 */
struct TokenStructure
{
  // Declaring token variables for each topics
};
// Declaring TokenStructure with token values for each topics
TokenStructure Tokenstruct;

/*!
 * \struct BufferDataStructure
 * \brief Structure containing buffered topics values of the Prod node 
 *
 */
struct BufferDataStructure
{
  // Declaring variables containing buffered topics values
  // Subscribers values
};
BufferDataStructure BufferDatastruct;


/*!
 * \fn int firing_node (void)
 * \brief This function is called to check if the node meeting conditions to fire the Prod_loop function
 *
 * \return The number of firing done.
 */
unsigned long Nb_jobs = 0;
int firing_node (void)
{
  int nb_firing = 0;
  if (\
  1)
  {
    if (Nb_jobs == 0)
    {
      // Polygraph trace specify user friendly name for thread
      polytef_thread_name("Prod");
    }
    
    // Filling User Datastruct and remove it from buffer

    // Polygraph trace start job
    polytef_job_start("Prod", Nb_jobs + 1);
    
    // Calling user loop
    Prod_loop (&Datastruct);

    // Publishing all topics
    polytef_channel_send("prod_out", Nb_jobs + 1);
    Prod_out_pub.publish(Datastruct.prod_out_msg);
    nb_firing++;
    
    // Polygraph trace job completed
    polytef_job_finish();
    Nb_jobs++;
  }
  return (nb_firing);
}


// Code specific to clocked node
// Timer declaration
ros::Timer Prod_timer;
// Start flag declaration
bool SystemReady = false;
/*!
 * \fn void sys_trigger_callback(const std_msgs::String::ConstPtr& msg)
 * \brief Callback function to start the Prod clocked node
 *
 * \param[in] msg Containing the value of the received topic
 *
 */
void sys_trigger_callback(const std_msgs::String::ConstPtr& msg)
{
  printf("Prod_node started ! - %s\n", msg->data.c_str());
  // Waiting Trigger
  SystemReady = true;
  // Adding phase
  ros::Duration(0.0).sleep();

  // Start timer
  Prod_timer.start();

  // Sending first iteration to the user
  while(0 == firing_node() && ros::ok())
  {
    ros::Duration(0,0005).sleep();
    ros::spinOnce();
  }
}

/*!
 * \fn void timer_callback(const ros::TimerEvent& event)
 * \brief Callback timer function of the Prod clocked node
 *
 * \param[in] event Containing the value of the current date and targeted date
 *
 */
void timer_callback(const ros::TimerEvent& event)
{
  // Firing clocked node until all inputs are available
  while(0 == firing_node() && ros::ok())
  {
    ros::Duration(0,0005).sleep();
    ros::spinOnce();
  }
}



/*!
 * \fn void init_buffer_token (void)
 * \brief Function to create valid token at initialization of Prod actor
 *
 */
void init_buffer_token (void)
{

}

/*!
 * \fn int main (int argc, char **argv)
 * \brief Entry point of the Prod program
 *
 * \param[in] argc Containing number of arguments passed in the command line
 * \param[in] argv Containing arguments passed in the command line
 *
 * \return EXIT_SUCCESS - Program successfully executed.
 */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "Prod_node");
  ros::NodeHandle nh;
  
  // Initializing Polygraph traces
  polytef_init("Prod");
  
  // Initializing buffer with init values
  init_buffer_token ();
  
  // Calling user setup
  Prod_setup (&Datastruct);

  // Creating Prod timer
  Prod_timer = nh.createTimer(ros::Duration(1./10), timer_callback, false, false);
  
  // Creating global system trigger
  ros::Subscriber sys_trig_sub = nh.subscribe("/sys_trigger", 1, sys_trigger_callback);

  // Creating Prod subscribers

  // Creating Prod publishers
  Prod_out_pub = nh.advertise<std_msgs::String>("/prod_out", 1);

  // While system is OK
  ros::spin();

  // Stopping timer because system is OFF
  Prod_timer.stop();

  // Finilize Polygraph traces
  polytef_finalize();

  return 0;
}