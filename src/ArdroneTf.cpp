/*
* ArdroneTf.cpp
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#include "ArdroneTf.h"

ArdroneTf::ArdroneTf(const char* file_name) : _file_path(file_name) {
  _cur_number = 0;
  _num_distance.push_back(_distance(1.9, -1.83));
  _num_distance.push_back(_distance(0.2, 3.3));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _num_distance.push_back(_distance(0, 0));
  _log.open(file_name);
  if (!_log) {
    cout << "ArdroneTf cannot open file to log" << endl;
  }
}

ArdroneTf::~ArdroneTf() {}

tf::StampedTransform ArdroneTf::get_transform(const char* frame1,
  const char* frame2) {
  
  tf::StampedTransform trans;
  try {
    ros::Time now = ros::Time::now();
    _listener.waitForTransform(frame1, frame2,
      now, ros::Duration(1.0));

    _listener.lookupTransform(frame1, frame2,
        now, trans);

  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  return trans;
}

void ArdroneTf::SetRefPose(double angle_offset) {
  _ref_trans = this->get_transform("odom", "ardrone_base_link");
  tf::Quaternion ref_qua = _ref_trans.getRotation();
  double yaw_plane, pitch, roll;
  _ref_trans.getBasis().getEulerYPR(yaw_plane, pitch, roll);

  // Two way of setting referance pose
  ref_qua.setEulerZYX(yaw_plane - angle_offset, 0.0, 0.0);
  //ref_qua.setEulerZYX(yaw_plane - angle_offset, pitch, roll)

  tf::Transform input(ref_qua, _ref_trans.getOrigin());
  _ref_trans.setData(input);
  if (_cur_number == 0) {
    _br.NewBr(input, "odom", "ref_pose");
  }
  else {
    _br.SetRefPose(input, "odom", "ref_pose");
  }

  _cur_number++;
  //Log Info
  LogCurTime();
  _log << "set number" << _cur_number << "'s referace pose" << endl;
  double yaw, pitch, roll;
  _ref_trans.getBasis().getEulerYPR(yaw, pitch, roll);
  _log << "Tx = " << _ref_trans.getOrigin().x()
       << " Ty = " << _ref_trans.getOrigin().y()
       << " Tz = " << _ref_trans.getOrigin().z() << endl;

  _log << "yaw = " << yaw << " pitch = " << pitch << " roll = " << roll << endl;
}


double ArdroneTf::YawDiff() {
  double yaw, yaw_ref, yaw_diff;
  double pitch, roll;
  this->get_transform("odom", "ardrone_base_link").getBasis().getEulerYPR(
      yaw, pitch, roll);

  _ref_trans.getBasis().getEulerYPR(yaw_ref, pitch, roll);
  //mat.getEulerYPR(yaw_ref, pitch, roll);
  yaw_diff = yaw - yaw_ref;
  if (yaw_diff < -PI) {
    yaw_diff += 2 * PI;
  }
  else if (yaw_diff > PI) {
    yaw_diff -= 2 * PI;
  }
  //Log Info
  LogCurTime();
  _log << "pitch_ref = " << pitch << "  roll_ref = " << roll << endl;
  _log << "yaw = " << yaw << "  yaw_ref = " << yaw_ref << endl;
  return yaw_diff;
}

double ArdroneTf::XDiff() {
  return - this->get_transform("ref_pose", "ardrone_base_link").getOrigin().x()
      - _num_distance[_cur_number-1]._x;

}

double ArdroneTf::YDiff() {
  return - this->get_transform("ref_pose", "ardrone_base_link").getOrigin().y() 
      - _num_distance[_cur_number-1]._y;

}

//void ArdroneTf::SetRefQuaternion() {
//  _ground_quaternion = 
//      this->get_transform("odom", "ardrone_base_link").getRotation();
//
//}

int ArdroneTf::get_cur_number() {
  return _cur_number;
}

void ArdroneTf::LogCurTime() {
  time_t timep;
  struct tm *a;
  time(&timep);
  a = localtime(&timep);
  _log << endl << a->tm_mday << " " << a->tm_hour << ":"
       << a->tm_min << ":" << a->tm_sec << endl;

}
