#include "trajectory_control_cartesian.h"

void trajectory_control::init()
{
  int argc=0;
  char** argv;
  ros::init(argc, argv, "trajectory_control_node");
  ros::NodeHandle n;

  psmPub.resize(6);
  cmd_joints.resize(6);
  cur_joints.resize(6);
  psmPub[0]=n.advertise<std_msgs::Float64>("/dvrk_psm/PSM1/yaw_joint/SetPositionTarget",1);
  psmPub[1]=n.advertise<std_msgs::Float64>("/dvrk_psm/PSM1/pitch_back_joint/SetPositionTarget",1);
  psmPub[2]=n.advertise<std_msgs::Float64>("/dvrk_psm/PSM1/main_insertion_joint/SetPositionTarget",1);
  psmPub[3]=n.advertise<std_msgs::Float64>("/dvrk_psm/PSM1/tool_roll_joint/SetPositionTarget",1);
  psmPub[4]=n.advertise<std_msgs::Float64>("/dvrk_psm/PSM1/tool_yaw_joint/SetPositionTarget",1);
  psmPub[5]=n.advertise<std_msgs::Float64>("/dvrk_psm/PSM1/tool_pitch_joint/SetPositionTarget",1);

  link_states=n.subscribe("/gazebo/link_states", 1, &trajectory_control::getBaseTransforms, this);
  joint_states=n.subscribe("/dvrk_psm/joint/states", 1, &trajectory_control::getCurrentJoints, this);
  subscribed = 0;
  psm_base = Eigen::Matrix4d::Identity();
  psm_end = Eigen::Matrix4d::Identity();
  // base_rcm << 0.0000, 1.0000, 0.0000, 0.0000,
  //            -1.0000, 0.0000, 0.0000, 0.4864,
  //             0.0000, 0.0000, 1.0000, 0.0000,
  //             0.0000, 0.0000, 0.0000, 1.0000;

  std::string filename = ros::package::getPath("trajectory_control");
  filename.append("/config/dvpsm.rob");
  result = psm_manip.LoadRobot(filename);

  while (subscribed<2)
    ros::spinOnce();

  calculate_rcm();

  cmd_joints[0].data = 0;
  cmd_joints[1].data = 0;
  cmd_joints[2].data = 0.12;
  cmd_joints[3].data = 0;
  cmd_joints[4].data = 0;
  cmd_joints[5].data = 0;

  moveRobot();

  // m_thread=new boost::thread(&trajectory_control::joint_space_trajectory, this);
  m_thread=new boost::thread(&trajectory_control::cartesian_space_trajectory, this);
}

void trajectory_control::getBaseTransforms(const gazebo_msgs::LinkStatesPtr &msg)
{
  // std::cout << "Subscribing..." << '\n';
  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare("dvrk_psm::PSM1::base_link"))
    {
      psm_base_gm = msg->pose[i];
      LinkStateToEigen(psm_base, psm_base_gm);
    }
    if (!msg->name[i].compare("dvrk_psm::PSM1::end_effector_Link"))
    {
      psm_end_gm = msg->pose[i];
      LinkStateToEigen(psm_end, psm_end_gm);
    }
  }
  subscribed = subscribed+1;
}

void trajectory_control::getCurrentJoints(const sensor_msgs::JointStatePtr &msg)
{
  sim_time = msg->header.stamp;
  for (int i=0;i<msg->name.size();i++)
  {
    if (!msg->name[i].compare("dvrk_psm/PSM1/yaw_joint"))
    {
      cur_joints[0].data = msg->position[i];
    }
    if (!msg->name[i].compare("dvrk_psm/PSM1/pitch_back_joint"))
    {
      cur_joints[1].data = msg->position[i];
    }
    if (!msg->name[i].compare("dvrk_psm/PSM1/main_insertion_joint"))
    {
      cur_joints[2].data = msg->position[i];
    }
    if (!msg->name[i].compare("dvrk_psm/PSM1/tool_roll_joint"))
    {
      cur_joints[3].data = msg->position[i];
    }
    if (!msg->name[i].compare("dvrk_psm/PSM1/tool_yaw_joint"))
    {
      cur_joints[4].data = msg->position[i];
    }
    if (!msg->name[i].compare("dvrk_psm/PSM1/tool_pitch_joint"))
    {
      cur_joints[5].data = msg->position[i];
    }
  }
  subscribed = subscribed+1;
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

void trajectory_control::calculate_rcm()
{
  vctDynamicVector<double> q(6,cur_joints[0].data, cur_joints[1].data,cur_joints[2].data,cur_joints[3].data,cur_joints[4].data,cur_joints[5].data);
  vctFrame4x4<double> psm_rcm_tip;
  psm_rcm_tip = psm_manip.ForwardKinematics(q);

  Eigen::Matrix4d rcm_tip = Eigen::Matrix4d::Identity();
  rcm_tip(0,0) = psm_rcm_tip[0][0];
  rcm_tip(0,1) = psm_rcm_tip[0][1];
  rcm_tip(0,2) = psm_rcm_tip[0][2];
  rcm_tip(0,3) = psm_rcm_tip[0][3];
  rcm_tip(1,0) = psm_rcm_tip[1][0];
  rcm_tip(1,1) = psm_rcm_tip[1][1];
  rcm_tip(1,2) = psm_rcm_tip[1][2];
  rcm_tip(1,3) = psm_rcm_tip[1][3];
  rcm_tip(2,0) = psm_rcm_tip[2][0];
  rcm_tip(2,1) = psm_rcm_tip[2][1];
  rcm_tip(2,2) = psm_rcm_tip[2][2];
  rcm_tip(2,3) = psm_rcm_tip[2][3];

  base_rcm = (psm_base.inverse())*psm_end*(rcm_tip.inverse());
  // std::cout << base_rcm << '\n';
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
  filename.append("/data/joint_space.csv");
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
    // cmd_joints[0].data = 1*sin((PI*t)/5.0);
    // cmd_joints[1].data = 0.85*sin((PI*t)/5.0);
    // cmd_joints[2].data = 0.12;
    // cmd_joints[3].data = 1*sin((PI*t)/5.0);
    // cmd_joints[4].data = 1*sin((PI*t)/5.0);
    // cmd_joints[5].data = 1*sin((PI*t)/5.0);

    cmd_joints[0].data = 0;
    cmd_joints[1].data = 0;
    cmd_joints[2].data = 0;
    cmd_joints[3].data = 0;
    cmd_joints[4].data = 0;
    cmd_joints[5].data = 0;

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

  // std::vector<std_msgs::Float64> calcu_joints = this->inverseKinematics((this->psm_base*this->base_rcm).inverse()*this->psm_end);
  // std::cout << calcu_joints[0].data << '\n';
  // std::cout << calcu_joints[1].data << '\n';
  // std::cout << calcu_joints[2].data << '\n';
  // std::cout << calcu_joints[3].data << '\n';
  // std::cout << calcu_joints[4].data << '\n';
  // std::cout << calcu_joints[5].data << '\n';

  std::cout << "Thread Ended" << '\n';
  dataFile.close();
}

void trajectory_control::cartesian_space_trajectory()
{
  std::vector<std_msgs::Float64> calc_joints;
  Eigen::MatrixXd coeff_x(6,1), coeff_y(6,1), coeff_z(6,1);
  sleep(10);
  m_thread_mutex->lock();
  Eigen::Matrix4d cmd_pose = this->psm_end;


  coeff_x = quintic_polynomial_gen(this->psm_end(0,3),0.05,0,0,0,0,0,10);
  coeff_y = quintic_polynomial_gen(this->psm_end(1,3),this->psm_end(1,3),0,0,0,0,0,10);
  coeff_z = quintic_polynomial_gen(this->psm_end(2,3),this->psm_end(2,3),0,0,0,0,0,10);
  m_thread_mutex->unlock();

  ros::Time start_time = ros::Time::now();
  ros::Time cur_time = ros::Time::now();

  std::ofstream dataFile;
  std::string filename = ros::package::getPath("trajectory_control");
  filename.append("/data/cartesian_space.csv");
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
    cmd_pose(0,3) = coeff_x(0,0) + t*coeff_x(1,0) + pow(t,2)*coeff_x(2,0) + pow(t,3)*coeff_x(3,0) + pow(t,4)*coeff_x(4,0) +pow(t,5)*coeff_x(5,0);
    cmd_pose(1,3) = coeff_y(0,0) + t*coeff_y(1,0) + pow(t,2)*coeff_y(2,0) + pow(t,3)*coeff_y(3,0) + pow(t,4)*coeff_y(4,0) +pow(t,5)*coeff_y(5,0);
    cmd_pose(2,3) = coeff_z(0,0) + t*coeff_z(1,0) + pow(t,2)*coeff_z(2,0) + pow(t,3)*coeff_z(3,0) + pow(t,4)*coeff_z(4,0) +pow(t,5)*coeff_z(5,0);

    calc_joints = this->inverseKinematics((this->psm_base*this->base_rcm).inverse()*cmd_pose);
    cmd_joints = calc_joints;
    this->moveRobot();
    r.sleep();

    dataFile << cur_time <<'\t';
    dataFile << cmd_pose(0,3) <<'\t';
    dataFile << cmd_pose(1,3) <<'\t';
    dataFile << cmd_pose(2,3) <<'\t';
    // dataFile << cmd_joints[3].data <<'\t';
    // dataFile << cmd_joints[4].data <<'\t';
    // dataFile << cmd_joints[5].data <<'\t';

    m_thread_mutex->lock();
    dataFile << sim_time <<'\t';
    dataFile << psm_end(0,3) <<'\t';
    dataFile << psm_end(1,3) <<'\t';
    dataFile << psm_end(2,3) <<'\t';
    // dataFile << cur_joints[3].data <<'\t';
    // dataFile << cur_joints[4].data <<'\t';
    // dataFile << cur_joints[5].data <<'\t';
    m_thread_mutex->unlock();

    dataFile << '\n';
  }
  std::cout << "Thread Ended" << '\n';
  dataFile.close();

}

Eigen::MatrixXd trajectory_control::quintic_polynomial_gen(double q_i, double q_f,double v_i, double v_f,double a_i, double a_f,double t_i, double t_f)
{
  Eigen::MatrixXd A(6,6), B(6,1), X(6,1);
  A << 1,t_i, pow(t_i,2),pow(t_i,3),pow(t_i,4),pow(t_i,5),
      0,1,2*t_i,3*pow(t_i,2),4*pow(t_i,3),5*pow(t_i,4),
      0,0,2,6*t_i,12*pow(t_i,2),20*pow(t_i,3),
      1,t_f, pow(t_f,2),pow(t_f,3),pow(t_f,4),pow(t_f,5),
      0,1,2*t_f,3*pow(t_f,2),4*pow(t_f,3),5*pow(t_f,4),
      0,0,2,6*t_f,12*pow(t_f,2),20*pow(t_f,3);
  B << q_i,
       v_i,
       a_i,
       q_f,
       v_f,
       a_f;

  X = A.inverse()*B;
  return X;
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


int main(int argc, char **argv)
{
  trajectory_control obj;
  // std::cout << (obj.psm_base*obj.base_rcm).inverse()*obj.psm_end << '\n';
  // std::cout << pcm_rcm_tip << '\n';
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
