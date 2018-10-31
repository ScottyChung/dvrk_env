#include "trajectory_control.h"

void trajectory_control::init(std::string rbt_namespace, std::string man_name)
{
    robot_namespace = rbt_namespace;
    manipulator_name = man_name;
  ros::NodeHandle n;

  psmPub.resize(6);
  cmd_joints.resize(6);
  cur_joints.resize(6);
  psmPub[0]=n.advertise<std_msgs::Float64>("/" + robot_namespace + "/" + manipulator_name + "/yaw_joint/SetPositionTarget",1);
  psmPub[1]=n.advertise<std_msgs::Float64>("/" + robot_namespace + "/" + manipulator_name + "/pitch_back_joint/SetPositionTarget",1);
  psmPub[2]=n.advertise<std_msgs::Float64>("/" + robot_namespace + "/" + manipulator_name + "/main_insertion_joint/SetPositionTarget",1);
  psmPub[3]=n.advertise<std_msgs::Float64>("/" + robot_namespace + "/" + manipulator_name + "/tool_roll_joint/SetPositionTarget",1);
  psmPub[4]=n.advertise<std_msgs::Float64>("/" + robot_namespace + "/" + manipulator_name + "/tool_yaw_joint/SetPositionTarget",1);
  psmPub[5]=n.advertise<std_msgs::Float64>("/" + robot_namespace + "/" + manipulator_name + "/tool_pitch_joint/SetPositionTarget",1);

  link_states=n.subscribe("/gazebo/link_states", 1, &trajectory_control::getBaseTransforms, this);
  joint_states=n.subscribe("/" + robot_namespace + "/joint/states", 1, &trajectory_control::getCurrentJoints, this);
  subscribed = 0;
  psm_base = Eigen::Matrix4d::Identity();
  psm_end = Eigen::Matrix4d::Identity();
  base_rcm << 0.0000, 1.0000, 0.0000, 0.4864,
             -1.0000, 0.0000, 0.0000, 0.0000,
              0.0000, 0.0000, 1.0000, 0.1524,
              0.0000, 0.0000, 0.0000, 1.0000;

  std::string filename = ros::package::getPath("trajectory_control");
  filename.append("/config/dvpsm.rob");
  result = psm_manip.LoadRobot(filename);

  while (!subscribed)
    ros::spinOnce();

  m_thread=new boost::thread(&trajectory_control::joint_space_trajectory, this);
}

void trajectory_control::getBaseTransforms(const gazebo_msgs::LinkStatesPtr &msg)
{
  subscribed = 1;
  // std::cout << "Subscribing..." << '\n';
  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare(robot_namespace + "::" + manipulator_name + "::base_link"))
    {
      psm_base_gm = msg->pose[i];
      LinkStateToEigen(psm_base, psm_base_gm);
    }
    if (!msg->name[i].compare(robot_namespace + "::" + manipulator_name + "::tool_gripper2_link"))
    {
      psm_end_gm = msg->pose[i];
      LinkStateToEigen(psm_end, psm_end_gm);
    }
  }
}

void trajectory_control::getCurrentJoints(const sensor_msgs::JointStatePtr &msg)
{
  sim_time = msg->header.stamp;
  for (int i=0;i<msg->name.size();i++)
  {
    if (!msg->name[i].compare(robot_namespace + "/" + manipulator_name + "/yaw_joint"))
    {
      cur_joints[0].data = msg->position[i];
    }
    if (!msg->name[i].compare(robot_namespace + "/" + manipulator_name + "/pitch_back_joint"))
    {
      cur_joints[1].data = msg->position[i];
    }
    if (!msg->name[i].compare(robot_namespace + "/" + manipulator_name + "/main_insertion_joint"))
    {
      cur_joints[2].data = msg->position[i];
    }
    if (!msg->name[i].compare(robot_namespace + "/" + manipulator_name + "/tool_roll_joint"))
    {
      cur_joints[3].data = msg->position[i];
    }
    if (!msg->name[i].compare(robot_namespace + "/" + manipulator_name + "/tool_yaw_joint"))
    {
      cur_joints[4].data = msg->position[i];
    }
    if (!msg->name[i].compare(robot_namespace + "/" + manipulator_name + "/tool_pitch_joint"))
    {
      cur_joints[5].data = msg->position[i];
    }
  }

}


void trajectory_control::LinkStateToEigen(Eigen::Matrix4d &mat, const geometry_msgs::Pose msg)
{
  // std::cout << "in link state" << '\n';
  mat(0,3) = msg.position.x;
  mat(1,3) = msg.position.y;
  mat(2,3) = msg.position.z;

  Eigen::Quaterniond q(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
  Eigen::Matrix3d R = q.toRotationMatrix();
  mat(0,0)=R(0,0);
  mat(0,1)=R(0,1);
  mat(0,2)=R(0,2);
  mat(1,0)=R(1,0);
  mat(1,1)=R(1,1);
  mat(1,2)=R(1,2);
  mat(2,0)=R(2,0);
  mat(2,1)=R(2,1);
  mat(2,2)=R(2,2);
  // std::cout << "out of link state" << '\n';
}

std::vector<std_msgs::Float64> trajectory_control::inverseKinematics(Eigen::Matrix4d base_to_end)
{
  std::vector<std_msgs::Float64> calc_joints;
  calc_joints.resize(6);
  vctDoubleVec psm_joint_calculated;
  vctFrm4x4 psm_pose_current;
  for (int i=0;i<4;i++)
  {
    for (int j=0;j<4;j++)
    {
      psm_pose_current[i][j]=base_to_end(i,j);
    }
  }
  psm_joint_calculated.SetSize(6);
  psm_joint_calculated[0]=0;
  psm_joint_calculated[1]=0;
  psm_joint_calculated[2]=0;
  psm_joint_calculated[3]=0;
  psm_joint_calculated[4]=0;
  psm_joint_calculated[5]=0;

  psm_manip.InverseKinematics(psm_joint_calculated, psm_pose_current);

  for (int i=0; i<6; i++)
  {
    calc_joints[i].data = psm_joint_calculated[i];
  }
  return calc_joints;
}

bool trajectory_control::RobotLoaded()
{
  return !result;
}

void trajectory_control::joint_space_trajectory()
{
  ros::Time start_time = ros::Time::now();
  ros::Time cur_time = ros::Time::now();

  std::ofstream dataFile;
  std::string filename = ros::package::getPath("trajectory_control");
  filename.append("/data/" + manipulator_name  +"joint_space.csv");
  // char file_str[100];
  // file_str=filename;
  // std::cout << file_str << '\n';
  dataFile.open(filename.c_str());
  if (!dataFile.is_open())
  {
    std::cout << "Data is not being saved" << '\n';
  }

  double t;
  // while
  ros::Rate r(1000);
  while (ros::ok() && ((cur_time - start_time).sec<10))
  {
    cur_time = ros::Time::now();
    t = cur_time.toSec() - start_time.toSec();
    // std::cout << t << '\n';
    // boost::this_thread::sleep_for(boost::chrono::microseconds(100));
    cmd_joints[0].data = 1*sin((PI*t)/5.0);
    cmd_joints[1].data = 0.85*sin((PI*t)/5.0 + 0.25*PI);
    cmd_joints[2].data = 0.10*sin((PI*t)/5.0 + PI) + 0.12;
    cmd_joints[3].data = 1*sin((PI*t)/5.0+0.5*PI);
    cmd_joints[4].data = 1*sin((PI*t)/5.0+0.75*PI);
    cmd_joints[5].data = 1*sin((PI*t)/5.0+1*PI);

    this->moveRobot();
    r.sleep();
    dataFile << cur_time <<'\t';
    dataFile << cmd_joints[0].data <<'\t';
    dataFile << cmd_joints[1].data <<'\t';
    dataFile << cmd_joints[2].data <<'\t';
    dataFile << cmd_joints[3].data <<'\t';
    dataFile << cmd_joints[4].data <<'\t';
    dataFile << cmd_joints[5].data <<'\t';

    m_thread_mutex->lock();
    dataFile << sim_time <<'\t';
    dataFile << cur_joints[0].data <<'\t';
    dataFile << cur_joints[1].data <<'\t';
    dataFile << cur_joints[2].data <<'\t';
    dataFile << cur_joints[3].data <<'\t';
    dataFile << cur_joints[4].data <<'\t';
    dataFile << cur_joints[5].data <<'\t';
    m_thread_mutex->unlock();

    dataFile << '\n';
  //   // std::cout << cur_time << '\n';
  }
  std::cout << "Thread Ended" << '\n';
  dataFile.close();
}

void trajectory_control::moveRobot()
{
  // psmPub[4].publish(cmd_joints[4]);
  // std::cout << ros::ok() << '\n';
  for (int i = 0;i<6;i++)
  {
    psmPub[i].publish(cmd_joints[i]);
  }
}

void trajectory_control::print()
{
  std::cout << psm_base << '\n';
}


//int main(int argc, char **argv)
//{
//  trajectory_control obj;
//  // std::cout << (obj.psm_base*obj.base_rcm).inverse()*obj.psm_end << '\n';
//  // std::cout << obj.RobotLoaded() << '\n';
//  // std::vector<std_msgs::Float64> calc_joints = obj.inverseKinematics((obj.psm_base*obj.base_rcm).inverse()*obj.psm_end);
//  // std::cout << calc_joints[0].data << '\n';
//  // std::cout << calc_joints[1].data << '\n';
//  // std::cout << calc_joints[2].data << '\n';
//  // std::cout << calc_joints[3].data << '\n';
//  // std::cout << calc_joints[4].data << '\n';
//  // std::cout << calc_joints[5].data << '\n';
//  ros::spin();
//
//}
