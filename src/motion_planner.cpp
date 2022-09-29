#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <planners/OMPLPlanner.hpp>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped goal;

    std::cout << "Enter goal x: ";
    std::cin >> goal.pose.position.x;
    std::cout << "Enter goal y: ";
    std::cin >> goal.pose.position.y;
    std::cout << "Enter goal z: ";
    std::cin >> goal.pose.position.z;

    OMPLPlanner planner(nh, goal);

    planner.navigate();

    return 0;
}
