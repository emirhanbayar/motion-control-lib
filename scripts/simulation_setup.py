#!/usr/bin/env python3
import os
import rospy
from pathlib import Path

str = """<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/mcmillian_airfield.world"/>
    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
    </include>

<!-- UAV0-->
    <group ns="uav0">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="0"/>
        <arg name="fcu_url" default="udp://:14540@localhost:14580"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="0"/>
            <arg name="y" value="0"/>
            <arg name="z" value="0"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            <arg name="mavlink_udp_port" value="14580@"/>
            <arg name="mavlink_tcp_port" value="4560@"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>
"""


obstacle_radii = rospy.get_param('obstacle_radii')
obstacle_heights = rospy.get_param('obstacle_heights')
obstacle_xs = rospy.get_param('obstacle_xs')
obstacle_ys = rospy.get_param('obstacle_ys')
obstacle_count = len(obstacle_radii)

for i in range(obstacle_count):
    urdf_file = Path(f"~/catkin_ws/src/quadcopter_motion_simulation/urdf/obstacle{i}.urdf").expanduser()
    urdf_file.touch(exist_ok=True)
    with urdf_file.open('w') as f:
        f.write("""<robot name="obstacle">
        <link name="my_obstacle">
            <inertial>
                <origin xyz="2 0 0" />
                <mass value="10.0" />
                <inertia  ixx="5.0" ixy="0.0"  ixz="0.0"  iyy="100.0"  iyz="0.0"  izz="1.0" />
            </inertial>
            <visual>
                <origin xyz="0 0 0"/>
                <geometry>
                <cylinder radius="{}" length="{}"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="2 0 1"/>
                <geometry>
                <box size="1 1 2" />
                </geometry>
            </collision>
        </link>
        <gazebo reference="my_obstacle">
            <material>Gazebo/Blue</material>
        </gazebo>
    </robot>""".format(obstacle_radii[i], obstacle_heights[i]))
    str += '<node name="obstacle_spawn{}" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -file {} -model my_obstacle{}  -x {} -y {} -z {}"/> \n'.format(i, urdf_file.absolute(), i, obstacle_xs[i], obstacle_ys[i], 0)
str += "</launch>"


f = open("../PX4-Autopilot/launch/multi_uav_mavros_sitl.launch", "w")
f.write(str)
f.close()
print("success")

os.system("source ../PX4-Autopilot/Tools/setup_gazebo.bash ../PX4-Autopilot/ ../PX4-Autopilot/build/px4_sitl_default")
os.system("export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:../PX4-Autopilot:../PX4-Autopilot/Tools/sitl_gazebo && roslaunch px4 multi_uav_mavros_sitl.launch")