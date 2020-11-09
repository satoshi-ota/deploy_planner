/*
* octomap_to_gridmap_demo_node.cpp
*
*  Created on: May 03, 2017
*      Author: Jeff Delmerico
*   Institute: University of Zürich, Robotics and Perception Group
*/

#include <ros/ros.h>
#include "deploy_planner/deploy_planner.h"

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "deploy_planner_node");
    ros::NodeHandle nh;
    DeployPlanner deploy_planner_node(nh);
    ros::Duration(2.0).sleep();
    ros::spin();
    return 0;
}
