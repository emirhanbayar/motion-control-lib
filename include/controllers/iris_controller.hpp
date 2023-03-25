#ifndef IRIS_CONTROLLER_HPP
#define IRIS_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


class iris_controller
{
private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Publisher waypoint_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    ros::Subscriber waypoint_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Rate rate = 20.0;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped waypoint;
    mavros_msgs::State current_state;

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void waypoint_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    int pose_error = 0.01;
public:
    iris_controller(ros::NodeHandle nh, ros::Rate rate);
    ~iris_controller();
    void go(geometry_msgs::PoseStamped target);
    void takeoff(int altitude);
    void land();
    geometry_msgs::PoseStamped getPose();
    
};

iris_controller::iris_controller(ros::NodeHandle nh, ros::Rate rate = 20.0)
{
    this->nh = nh;
    this->rate = rate;

    // Subscriber for getting current pose of the iris
    this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose", 10, &iris_controller::pose_cb, this);
    
    // Subscriber for getting current state of the iris. This is used to check if the iris is connected.
    this->state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, &iris_controller::state_cb, this);

    // Subscriber for getting the current waypoint from the planner
    this->waypoint_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/waypoints", 10, &iris_controller::waypoint_cb, this);
    
    // Service for sending the arming (getting the iris drone ready for commands) command to the iris
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");

    // Service for sending the set OFFBOARD mode command to the iris
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    
    // Publisher for sending the current pose of the iris to the planner
    this->pose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);

    // Publisher for sending the current waypoint to the iris
    this->waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/waypoints", 10);
}

iris_controller::~iris_controller()
{
}

void iris_controller::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    // Callback function for the state subscriber
    this->current_state = *msg;
}

void iris_controller::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Callback function for the pose subscriber
    pose = *msg;
}

void iris_controller::waypoint_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Callback function for the waypoint subscriber
    waypoint = *msg;
}

float distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    return sqrt(pow(a.pose.position.x - b.pose.position.x, 2) + pow(a.pose.position.y - b.pose.position.y, 2) + pow(a.pose.position.z - b.pose.position.z, 2));
}

void iris_controller::go(geometry_msgs::PoseStamped target)
{
    // Iris controller is programmed to go to the current waypoint. Thus, publishing the target as a waypoint ensures that the iris will go to the target.
    waypoint_pub.publish(target);
}

void iris_controller::takeoff(int altitude)
{
    /* This is the function for taking off the iris drone, and for starting main loop of our controller               */
    /* In the main loop, the iris drone will go to the current waypoint, and will wait until it reaches the waypoint. */

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose_pub.publish(pose);
        waypoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set the iris drone to OFFBOARD mode. OFFBOARD mode is a mode in which the iris drone can be controlled by the controller. 
    //For more information, please refer to https://docs.px4.io/main/en/flight_modes/offboard.html
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Set the iris drone to armed mode. Armed mode is a mode in which the iris drone is ready to take off.
    // For more information, please refer to https://ardupilot.org/copter/docs/arming_the_motors.html
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // Main loop of the controller
    while(ros::ok()){
        
        // This is a trick to initialize the iris drone to the current waypoint. This is necessary because the iris drone will not go to the current waypoint if it is not initialized.
        waypoint_pub.publish(waypoint);

        // If the iris drone is not in OFFBOARD mode, send the OFFBOARD mode command to the iris drone
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

        // If the iris drone is not armed, send the arming command to the iris drone
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // If the iris drone is in OFFBOARD mode and is armed, send the current waypoint to the iris drone
        pose_pub.publish(waypoint);


        // If the iris drone is within 0.1 meters of the current waypoint, stop the iris drone
        ros::spinOnce();
        rate.sleep();
    }
}

geometry_msgs::PoseStamped iris_controller::getPose()
{
    return pose;
}

#endif