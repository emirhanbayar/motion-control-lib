#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <controllers/iris_controller.hpp>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    iris_controller controller(nh);

    controller.takeoff(2);

    ros::spin();

    return 0;
}
