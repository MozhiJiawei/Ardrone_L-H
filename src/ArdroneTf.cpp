/*
* ArdroneTf.cpp
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#include "ArdroneTf.h"

ArdroneTf::ArdroneTf(const char* file_name) : _file_path(file_name) {
  _cur_number = 0;
  _num_distance[0] = _distance(0, 0);
  _num_distance[1] = _distance(0, 0);
  _num_distance[2] = _distance(0, 0);
  _num_distance[3] = _distance(0, 0);
  _num_distance[4] = _distance(0, 0);
  _num_distance[5] = _distance(0, 0);
  _num_distance[6] = _distance(0, 0);
  _num_distance[7] = _distance(0, 0);
  _num_distance[8] = _distance(0, 0);
}

ArdroneTf::~ArdroneTf() {}

tf::StampedTransform ArdroneTf::get_transform(const char* frame1, const char* frame2) {
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

void ArdroneTf::SetRefPose(int number) {
  tf::StampedTransform stam_trans;
  stam_trans = this->get_transform("odom", "ardrone_base_link");
  tf::Transform trans(stam_trans.getRotation(), stam_trans.getOrigin());
  _broadcaster.sendTransform(
      tf::StampedTransform(trans, ros::Time::now(), "odom", "ref_pose"));

  _cur_number = number;
}

double ArdroneTf::YawDiff() {
  double yaw, pitch, roll;
  tf::StampedTransform stam_trans;
  stam_trans = this->get_transform("odom", "ardrone_base_link");

  stam_trans.getBasis().getEulerYPR(yaw, pitch, roll);
  return yaw;
}

double ArdroneTf::XDiff() {
  tf::StampedTransform stam_trans;
  stam_trans = this->get_transform("ref_pose", "ardrone_base_link");
  return stam_trans.getOrigin().x() - _num_distance[_cur_number]._x;
}

double ArdroneTf::YDiff() {
  tf::StampedTransform stam_trans;
  stam_trans = this->get_transform("ref_pose", "ardrone_base_link");
  return stam_trans.getOrigin().y() - _num_distance[_cur_number]._y;
}
