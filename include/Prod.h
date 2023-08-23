/*!
 * \file Prod.h
 * \brief Header for the Prod node
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 1.0
 * \date 05/07/2023
 *
 * Header for the Prod node - generated automatically (contact A. Berne if errors)
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 */

#ifndef PROD_H
#define PROD_H

#include "ros/ros.h"
// Including headers for node topics
#include "std_msgs/String.h"


/*!
 * \struct DataStructure
 * \brief Structure containing topic values of the Prod node 
 *
 * This structure is declared for each nodes
 * Be carefull because it is using the same name for every nodes
 */
struct DataStructure
{
  // Declaring variables containing topics values
  // Subscribers values
  // Publishers values
  std_msgs::String prod_out_msg;
};

// Declaring DataStructure with all topics values
extern DataStructure Datastruct; // Declared in user code

#endif