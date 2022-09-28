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
    ros::Publisher velo_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Rate rate = 20.0;

    geometry_msgs::PoseStamped pose;
    mavros_msgs::State current_state;
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    
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

    this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose", 10, &iris_controller::pose_cb, this);
    this->state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, &iris_controller::state_cb, this);
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    this->velo_pub = nh.advertise<geometry_msgs::Twist>
            ("/uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
}

iris_controller::~iris_controller()
{
}

void iris_controller::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    this->current_state = *msg;
}

void iris_controller::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
}

int distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    return sqrt(pow(a.pose.position.x - b.pose.position.x, 2) + pow(a.pose.position.y - b.pose.position.y, 2) + pow(a.pose.position.z - b.pose.position.z, 2));
}

void iris_controller::go(geometry_msgs::PoseStamped target)
{
    geometry_msgs::Twist velo;
    int Kv = 1;
    while (ros::ok() && distance(pose, target) > pose_error)
    {
        velo.linear.x = Kv * (target.pose.position.x - pose.pose.position.x);
        velo.linear.y = Kv * (target.pose.position.y - pose.pose.position.y);
        velo.linear.z = Kv * (target.pose.position.z - pose.pose.position.z);
        velo_pub.publish(velo);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

void iris_controller::takeoff(int altitude)
{
    ROS_INFO("Connecting to FCU");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist velo;
    velo.linear.x = 0;
    velo.linear.y = 0;
    velo.linear.z = altitude;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ROS_INFO("arming");
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        velo.linear.x = 0;
        velo.linear.y = 0;
        velo.linear.z = altitude - pose.pose.position.z;
        velo_pub.publish(velo);
        

        ros::spinOnce();
        rate.sleep();
    }
}

void iris_controller::land()
{
    geometry_msgs::Twist velo;
    velo.linear.x = 0;
    velo.linear.y = 0;
    velo.linear.z = 0;
    velo_pub.publish(velo);
}

geometry_msgs::PoseStamped iris_controller::getPose()
{
    return pose;
}

#endif