/**
 * @file motion_planner.cpp
 * @brief Use OMPLPlanner class to find a path from start to goal avoiding obstacles. Then use the functions defined in iris_controller.hpp to follow the path.
 * @date 2022-09-28
 * 
 */
 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <planners/OMPLPlanner.hpp>
#include <controllers/iris_controller.hpp>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    iris_controller controller(nh);

    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;

    start = controller.getPose();

    start.pose.position.x = 0;
    start.pose.position.y = 0;
    start.pose.position.z = 0;

    goal.pose.position.x = 0.1;
    goal.pose.position.y = 0.1;
    goal.pose.position.z = 0.1;

    OMPLPlanner planner(nh, start, goal);

    planner.getWaypoints();

    ros::spin();

    return 0;
}
