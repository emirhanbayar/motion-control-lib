#ifndef OMPLPLANNER_HPP
#define OMPLPLANNER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/SpawnModel.h>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
 
#include <ompl/config.h>
#include <iostream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class OMPLPlanner
{
private:
    ros::NodeHandle nh;
    std::vector<double> obstacle_xs;
    std::vector<double> obstacle_ys;
    std::vector<double> obstacle_radii;
    std::vector<double> obstacle_heights;
    
    std::vector<geometry_msgs::PoseStamped> waypoints;
    
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;

    ros::Subscriber pose_sub;
    ros::Publisher waypoint_pub;

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool isStateValid(const ob::State *state);

public:
    OMPLPlanner(ros::NodeHandle nh, geometry_msgs::PoseStamped goal);
    ~OMPLPlanner();
    std::vector<geometry_msgs::PoseStamped>& getWaypoints();
    void navigate();
};

void OMPLPlanner::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    this->current_pose = *msg;
}

OMPLPlanner::OMPLPlanner(ros::NodeHandle nh, geometry_msgs::PoseStamped goal)
{
    this->nh = nh;
    this->goal = goal;
    this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/mavros/local_position/pose", 10, &OMPLPlanner::pose_cb, this);
    this->waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/waypoints", 10);

    while (this->current_pose.header.seq == 0)
    {
        ros::spinOnce();
    }
    this->start = current_pose;

    // Get obstacle parameters
    nh.getParam("obstacle_xs", obstacle_xs);
    nh.getParam("obstacle_ys", obstacle_ys);
    nh.getParam("obstacle_radii", obstacle_radii);
    nh.getParam("obstacle_heights", obstacle_heights);

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());
 
    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
 
    space->setBounds(bounds);
 
    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
 
    // set state validity checking for this space
    si->setStateValidityChecker(std::bind(&OMPLPlanner::isStateValid, this, std::placeholders::_1));
 
    // create a random start state
    ROS_INFO("start: %f, %f, %f", start.pose.position.x, start.pose.position.y, start.pose.position.z);
    ob::ScopedState<> ompl_start(space);
    ompl_start.random();
    ompl_start->as<ob::SE3StateSpace::StateType>()->setXYZ(start.pose.position.x, start.pose.position.y, start.pose.position.z);
    ompl_start->as<ob::SE3StateSpace::StateType>()->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    // create a random goal state
    ROS_INFO("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    ob::ScopedState<> ompl_goal(space);
    ompl_goal.random();
    ompl_goal->as<ob::SE3StateSpace::StateType>()->setXYZ(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    ompl_goal->as<ob::SE3StateSpace::StateType>()->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

 
    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    
    // set a path length optimization objective
    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(ompl_start, ompl_goal);
 
    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTstar>(si));
 
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    
    ompl_start.print(std::cout);
    // perform setup steps for the planner
    planner->setup();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(30);
 
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        
        //get waypoints from path
        std::vector<ob::State*> waypoints = path->as<og::PathGeometric>()->getStates();

        //print waypoints
        for (int i = 0; i < waypoints.size(); i++)
        {
            const ob::SE3StateSpace::StateType *se3state = waypoints[i]->as<ob::SE3StateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
            geometry_msgs::PoseStamped waypoint;
            waypoint.pose.position.x = pos->values[0];
            waypoint.pose.position.y = pos->values[1];
            waypoint.pose.position.z = pos->values[2];
            this->waypoints.push_back(waypoint);

            // // spawn waypoint in gazebo
            // std::string model_name = "waypoint" + std::to_string(i);
            // std::string model_path = "src/quadcopter_motion_simulation/urdf/waypoint.urdf";
            // // <node name="obstacle_spawn{}" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -file {} -model my_obstacle{}  -x {} -y {} -z {}"/> \n'.format(i, urdf_file.absolute(), i, obstacle_xs[i], obstacle_ys[i], 0)
            // std::string model_command = "rosrun gazebo_ros spawn_model -urdf -f " + model_path + " -m " + model_name + " -x " + std::to_string(pos->values[0]) + " -y " + std::to_string(pos->values[1]) + " -z " + std::to_string(pos->values[2]);
            // system(model_command.c_str());


            ROS_INFO("waypoint: %f, %f, %f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        }
    }


}

OMPLPlanner::~OMPLPlanner()
{
}

bool OMPLPlanner::isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot
    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    // check if the state is in collision with any of the obstacles
    for (int i = 0; i < obstacle_xs.size(); i++)
    {
        double x_diff = x - obstacle_xs[i];
        double y_diff = y - obstacle_ys[i];
        double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
        if ((z < obstacle_heights[i] && distance < obstacle_radii[i] + 0.3) || z < 0.1)
        {
            return false;
        }
    }
    return true;
}

std::vector<geometry_msgs::PoseStamped>& OMPLPlanner::getWaypoints()
{
    return waypoints;
}

float distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
    return sqrt(pow(a.pose.position.x - b.pose.position.x, 2) + pow(a.pose.position.y - b.pose.position.y, 2) + pow(a.pose.position.z - b.pose.position.z, 2));
}

void OMPLPlanner::navigate()
{
        for (int i = 0; i < waypoints.size(); i++)
        {   
            ROS_INFO("Going to waypoint %f %f %f", waypoints[i].pose.position.x, waypoints[i].pose.position.y, waypoints[i].pose.position.z);
            while(distance(waypoints[i], current_pose) > 0.1)
            {
                waypoint_pub.publish(waypoints[i]);
                ros::spinOnce();
            }
        }

        waypoint_pub.publish(goal);
}

#endif