#include <controllers/iris_controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff");
    ros::NodeHandle nh;

    iris_controller controller(nh);

    controller.takeoff(2);

    return 0;
}
