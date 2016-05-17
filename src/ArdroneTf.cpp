/*
* ArdroneTf.cpp
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#include "ArdroneTf.h"

ArdroneTf::ArdroneTf(const char* file_name) : _file_path(file_name) {
  _cur_number = 0;
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
}

ArdroneTf::~ArdroneTf() {}

tf::StampedTransform ArdroneTf::get_transform(const char* frame1,
  const char* frame2) {
  
  tf::StampedTransform trans;
  try {
    _listener.lookupTransform(frame1, frame2,
      ros::Time(0), trans);

  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  return trans;
}

void ArdroneTf::SetRefPose() {
  tf::StampedTransform stam_trans;
  stam_trans = this->get_transform("odom", "ardrone_base_link");
  tf::Transform trans(_ground_quaternion, stam_trans.getOrigin());
  _broadcaster.sendTransform(
      tf::StampedTransform(trans, ros::Time::now(), "odom", "ref_pose"));

  _cur_number++;
}

double ArdroneTf::YawDiff() {
  double yaw, yaw_ref, yaw_diff;
  double pitch, roll;
  tf::Matrix3x3 mat(_ground_quaternion);
  this->get_transform("odom", "ardrone_base_link").getBasis().getEulerYPR(
      yaw, pitch, roll);

  mat.getEulerYPR(yaw_ref, pitch, roll);
  yaw_diff = yaw - yaw_ref;
  if (yaw_diff < -PI) {
    yaw_diff += 2 * PI;
  }
  else if (yaw_diff > PI) {
    yaw_diff -= 2 * PI;
  }
  return yaw;
}

double ArdroneTf::XDiff() {
  return this->get_transform("ref_pose", "ardrone_base_link").getOrigin().x()
      - _num_distance[_cur_number]._x;

}

double ArdroneTf::YDiff() {
  return this->get_transform("ref_pose", "ardrone_base_link").getOrigin().y() 
      - _num_distance[_cur_number]._y;

}

void ArdroneTf::SetRefQuaternion() {
  _ground_quaternion = 
      this->get_transform("odom", "ardrone_base_link").getRotation();

}

int ArdroneTf::get_cur_number() {
  return _cur_number;
}

