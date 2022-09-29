#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <planners/OMPLPlanner.hpp>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped goal;

    goal.pose.position.x = 5;
    goal.pose.position.y = 5;
    goal.pose.position.z = 2;

    OMPLPlanner planner(nh, goal);

    planner.navigate();

    return 0;
}
