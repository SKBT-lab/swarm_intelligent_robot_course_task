#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <algorithm>

#include "Minimumsnap.hpp"

using namespace std;

quadrotor_msgs::PositionCommand posiCmd;
ros::Publisher controlCmd_pub;
double last_yaw_;
bool zoom_enable;
TRAJECTORY_GENERATOR tg;

double clamp(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}

std::pair<double, double> calculate_yaw(const Traj_info& t_info){
  Vector3d pos = t_info.position;
  Vector3d vel = t_info.velocity;
  Vector3d acc = t_info.acceleration;

  double dx = vel.x();
  double dy = vel.y();

  double ddx = acc.x();
  double ddy = acc.y();

  double yaw = atan(dy / dx);
  double dyaw = 1 / (1 + (dy / dx) * (dy / dx)) * (ddy / dx - dy * ddx * dx / (dx * dx));

  dyaw = clamp(dyaw, -1.5, 1.5);

  if(dx < 0){               //由于反正切的值域是-PI/2到PI/2,而dx<0的部分在该范围之外，故额外叠加个PI
        yaw = yaw - M_PI;   //根据任务流程，机体优先顺时针旋转，故减PI而不是加PI
    }

  std::pair<double, double> yaw_dyaw;
  yaw_dyaw.first = yaw;
  yaw_dyaw.second = dyaw;
  return yaw_dyaw;  
}


bool ControlParse(MiniSnapTraj trajectory, ros::Time start_time, bool init, bool isAdjustYaw) {
  // std::lock_guard<std::mutex> lock(csf_mutex);
  ros::Time time_now = ros::Time::now();
  // double t_cur = (time_now - start_time).toSec();
  double t_cur = (time_now - trajectory.start_time).toSec();
//   t_cur += time_forward;
//   if(zoom_enable && t_cur >= tv1 && t_cur < tv2 - dtv){
//     t_cur = tv1 + (t_cur - tv1) * zoom_factor;
//     // std::cout << "zoom!" << std::endl;
//   }
//   else if(zoom_enable && t_cur >= tv2 - dtv){
//     t_cur = t_cur + dtv;
//     // std::cout << "stop zoom!" << std::endl;
//   }

  bool finished = false;
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0); 
  static ros::Time time_last = ros::Time::now();

  if (init) {
    time_last = ros::Time::now();
  } 

  if (t_cur < trajectory.time_duration && t_cur >= 0.0){
    Traj_info info = tg.getTrajInfo(trajectory, t_cur);
    pos = info.position;
    vel = info.velocity;
    yaw_yawdot = calculate_yaw(info);
  }
  else if (t_cur >= trajectory.time_duration){
    Traj_info info = tg.getTrajInfo(trajectory, trajectory.time_duration);
    pos = info.position;
    vel.setZero();
    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;
    finished = true; 
  }
  else{
    cout << "[trajectory server]: invalid time." << endl;
  }

  time_last = time_now;

  posiCmd.header.stamp = time_now;
  posiCmd.header.frame_id = "world";
  posiCmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  posiCmd.trajectory_id = 1;

  posiCmd.position.x = pos(0);
  posiCmd.position.y = pos(1);
  posiCmd.position.z = pos(2);

  posiCmd.velocity.x = vel(0);
  posiCmd.velocity.y = vel(1);
  posiCmd.velocity.z = vel(2);

  if (isAdjustYaw) {
    posiCmd.yaw = yaw_yawdot.first;
  }
  else {
    posiCmd.yaw = last_yaw_;
  }
  
  // posiCmd.yaw_dot = yaw_yawdot.second;
  posiCmd.yaw_dot = 0;
  last_yaw_ = posiCmd.yaw;
  controlCmd_pub.publish(posiCmd);
  return finished;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "quad_sim_example");
  ros::NodeHandle nh("~");

  controlCmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/uav2/position_cmd", 10);

  ros::Duration(2.0).sleep();

  
  last_yaw_ = 0;
  zoom_enable = false;

  vector<Eigen::Vector3d> waypoints;

  waypoints.push_back(Eigen::Vector3d(-2, -2, 1));
  waypoints.push_back(Eigen::Vector3d(-10, 0, 5));
  waypoints.push_back(Eigen::Vector3d(-13, 5, 3));


  MiniSnapTraj Cur_Traj = tg.trajGeneration(waypoints, ros::Time::now());
  

  ros::Rate rate(10);

  ros::Time tra_start_time = ros::Time::now();
  while (ros::ok())
  {
    ControlParse(Cur_Traj, tra_start_time, true, true);
    rate.sleep();
  }

  return 0;
}