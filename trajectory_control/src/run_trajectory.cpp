#include "trajectory_control.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>

#include <sensor_msgs/Image.h>
#include <sstream>
#include <fstream>

#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <Eigen/Dense>

int main(int argc, char **argv)
{
    //int argc=0;
    //char** argv;
    ros::init(argc, argv, "trajectory_control_node");

    std::string rbt_namespace = "dvrk_psm";
    std::string psm = "PSM1";
    trajectory_control obj(rbt_namespace, psm);
    //trajectory_control obj2(rbt_namespace, "PSM2");
    //trajectory_control obj3(rbt_namespace, "PSM3");
    // std::cout << (obj.psm_base*obj.base_rcm).inverse()*obj.psm_end << '\n';
    // std::cout << obj.RobotLoaded() << '\n';
    // std::vector<std_msgs::Float64> calc_joints = obj.inverseKinematics((obj.psm_base*obj.base_rcm).inverse()*obj.psm_end);
    // std::cout << calc_joints[0].data << '\n';
    // std::cout << calc_joints[1].data << '\n';
    // std::cout << calc_joints[2].data << '\n';
    // std::cout << calc_joints[3].data << '\n';
    // std::cout << calc_joints[4].data << '\n';
    // std::cout << calc_joints[5].data << '\n';
    ros::spin();

}