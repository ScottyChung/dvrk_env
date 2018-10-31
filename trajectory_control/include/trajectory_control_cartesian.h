#ifndef TRAJECTORY_H
#define TRAJECTORY_H
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
#define PI 3.14159265359

class trajectory_control{
private:
  double subscribed;
  std::vector<ros::Publisher> psmPub;
  std::vector<std_msgs::Float64> cur_joints, cmd_joints;
  ros::Subscriber link_states, joint_states;
  geometry_msgs::Pose psm_base_gm, psm_end_gm;
  robManipulator psm_manip;
  robManipulator::Errno result;
  ros::Time sim_time;
  // ros::Time start_time, cur_time;
  boost::thread* m_thread;
  boost::mutex *m_thread_mutex = new boost::mutex;
  // std::vector<Float64> v;
public:
  Eigen::Matrix4d psm_base, psm_end, base_rcm;
  trajectory_control(){
    init();
  };

  void init();
  void LinkStateToEigen(Eigen::Matrix4d &mat, const geometry_msgs::Pose msg);
  void getBaseTransforms(const gazebo_msgs::LinkStatesPtr &msg);
  void calculate_rcm();
  void getCurrentJoints(const sensor_msgs::JointStatePtr &msg);
  std::vector<std_msgs::Float64> inverseKinematics(Eigen::Matrix4d base_to_end);
  bool RobotLoaded();
  void moveRobot();
  void joint_space_trajectory();
  void cartesian_space_trajectory();
  Eigen::MatrixXd quintic_polynomial_gen(double q_i, double q_f,double v_i, double v_f,double a_i, double a_f,double t_i, double t_f);
  void print();


};
#endif
