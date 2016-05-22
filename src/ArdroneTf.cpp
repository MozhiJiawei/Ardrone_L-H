/*
* ArdroneTf.cpp
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#include "ArdroneTf.h"

ArdroneTf::ArdroneTf(const char* file_name) : _file_path(file_name) {
  _cur_number = 0;
  _tar_number = 1;
  // 0:
  vector<_distance> dis_zero;
  dis_zero.push_back(_distance(0, 0)); //0->0
  dis_zero.push_back(_distance(1.9, -1.83)); //0->1
  dis_zero.push_back(_distance(0, 0)); //0->2
  dis_zero.push_back(_distance(2.73, -0.25)); //0->3
  dis_zero.push_back(_distance(0, 0)); //0->4
  dis_zero.push_back(_distance(0, 0)); //0->5
  dis_zero.push_back(_distance(0, 0)); //0->6
  dis_zero.push_back(_distance(0, 0)); //0->7
  dis_zero.push_back(_distance(0, 0)); //0->8
  dis_zero.push_back(_distance(0, 0)); //0->9
  _num_distance.push_back(dis_zero);

  // 1:
  vector<_distance> dis_one;
  dis_one.push_back(_distance(0, 0)); //1->0
  dis_one.push_back(_distance(0, 0)); //1->1
  dis_one.push_back(_distance(0.05, 3.0)); //1->2 0.05  3.3
  dis_one.push_back(_distance(0, 0)); //1->3
  dis_one.push_back(_distance(0, 0)); //1->4
  dis_one.push_back(_distance(0, 0)); //1->5
  dis_one.push_back(_distance(0, 0)); //1->6
  dis_one.push_back(_distance(0, 0)); //1->7
  dis_one.push_back(_distance(0, 0)); //1->8
  dis_one.push_back(_distance(0, 0)); //1->9
  _num_distance.push_back(dis_one);

  // 2:
  vector<_distance> dis_two;
  dis_two.push_back(_distance(0, 0)); //2->0
  dis_two.push_back(_distance(0, 0)); //2->1
  dis_two.push_back(_distance(0, 0)); //2->2
  dis_two.push_back(_distance(0.4, -1.6)); //2->3
  dis_two.push_back(_distance(0, 0)); //2->4
  dis_two.push_back(_distance(0, 0)); //2->5
  dis_two.push_back(_distance(0, 0)); //2->6
  dis_two.push_back(_distance(0, 0)); //2->7
  dis_two.push_back(_distance(0, 0)); //2->8
  dis_two.push_back(_distance(0, 0)); //2->9
  _num_distance.push_back(dis_two);

  // 3:
  vector<_distance> dis_three;
  dis_three.push_back(_distance(0, 0)); //3->0
  dis_three.push_back(_distance(-0.85, -1.65)); //3->1
  dis_three.push_back(_distance(-0.6, 1.6)); //3->2
  dis_three.push_back(_distance(0, 0)); //3->3
  dis_three.push_back(_distance(2.0, -1.5)); //3->4 1.9  -1.6
  dis_three.push_back(_distance(0, 0)); //3->5
  dis_three.push_back(_distance(0, 0)); //3->6
  dis_three.push_back(_distance(0, 0)); //3->7
  dis_three.push_back(_distance(0, 0)); //3->8
  dis_three.push_back(_distance(0, 0)); //3->9
  _num_distance.push_back(dis_three);

 // 4:
  vector<_distance> dis_four;
  dis_four.push_back(_distance(0, 0)); //4->0
  dis_four.push_back(_distance(0, 0)); //4->1
  dis_four.push_back(_distance(0, 0)); //4->2
  dis_four.push_back(_distance(0, 0)); //4->3
  dis_four.push_back(_distance(0, 0)); //4->4
  dis_four.push_back(_distance(2.96, 2.28)); //4->5
  dis_four.push_back(_distance(0, 0)); //4->6
  dis_four.push_back(_distance(0, 0)); //4->7
  dis_four.push_back(_distance(0, 0)); //4->8
  dis_four.push_back(_distance(0, 0)); //4->9
  _num_distance.push_back(dis_four);

  // 5:
  vector<_distance> dis_five;
  dis_five.push_back(_distance(0, 0)); //5->0
  dis_five.push_back(_distance(0, 0)); //5->1
  dis_five.push_back(_distance(0, 0)); //5->2
  dis_five.push_back(_distance(0, 0)); //5->3
  dis_five.push_back(_distance(0, 0)); //5->4
  dis_five.push_back(_distance(0, 0)); //5->5
  dis_five.push_back(_distance(-2.8, 0.6)); //5->6
  dis_five.push_back(_distance(2.07, 0.6)); //5->7
  dis_five.push_back(_distance(0, 0)); //5->8
  dis_five.push_back(_distance(0, 0)); //5->9
  _num_distance.push_back(dis_five);

  // 6:
  vector<_distance> dis_six;
  dis_six.push_back(_distance(0, 0)); //6->0
  dis_six.push_back(_distance(0, 0)); //6->1
  dis_six.push_back(_distance(0, 0)); //6->2
  dis_six.push_back(_distance(0, 0)); //6->3
  dis_six.push_back(_distance(0, 0)); //6->4
  dis_six.push_back(_distance(2.79, -0.5)); //6->5
  dis_six.push_back(_distance(0, 0)); //6->6
  dis_six.push_back(_distance(4.8, 0.1)); //6->7
  dis_six.push_back(_distance(0, 0)); //6->8
  dis_six.push_back(_distance(0, 0)); //6->9
  _num_distance.push_back(dis_six);

  // 7:
  vector<_distance> dis_seven;
  dis_seven.push_back(_distance(0, 0)); //7->0
  dis_seven.push_back(_distance(0, 0)); //7->1
  dis_seven.push_back(_distance(0, 0)); //7->2
  dis_seven.push_back(_distance(0, 0)); //7->3
  dis_seven.push_back(_distance(0, 0)); //7->4
  dis_seven.push_back(_distance(0, 0)); //7->5
  dis_seven.push_back(_distance(0, 0)); //7->6
  dis_seven.push_back(_distance(0, 0)); //7->7
  dis_seven.push_back(_distance(-0.6, -2.6)); //7->8
  dis_seven.push_back(_distance(0, 0)); //7->9
  _num_distance.push_back(dis_seven);

  // 8:
  vector<_distance> dis_eight;
  dis_eight.push_back(_distance(0, 0)); //8->0
  dis_eight.push_back(_distance(0, 0)); //8->1
  dis_eight.push_back(_distance(0, 0)); //8->2
  dis_eight.push_back(_distance(0, 0)); //8->3
  dis_eight.push_back(_distance(0, 0)); //8->4
  dis_eight.push_back(_distance(0, 0)); //8->5
  dis_eight.push_back(_distance(0, 0)); //8->6
  dis_eight.push_back(_distance(0, 0)); //8->7
  dis_eight.push_back(_distance(0, 0)); //8->8
  dis_eight.push_back(_distance(-3.45, 0.9)); //8->9
  _num_distance.push_back(dis_eight);

  // 9:
  vector<_distance> dis_nine;
  dis_nine.push_back(_distance(0, 0)); //9->0
  dis_nine.push_back(_distance(0, 0)); //9->1
  dis_nine.push_back(_distance(0, 0)); //9->2
  dis_nine.push_back(_distance(0, 0)); //9->3
  dis_nine.push_back(_distance(0, 0)); //9->4
  dis_nine.push_back(_distance(0, 0)); //9->5
  dis_nine.push_back(_distance(0, 0)); //9->6
  dis_nine.push_back(_distance(0, 0)); //9->7
  dis_nine.push_back(_distance(0, 0)); //9->8
  dis_nine.push_back(_distance(0, 0)); //9->9
  _num_distance.push_back(dis_nine);
/*
  _num_distance.push_back(_distance(1.9, -1.83));
  _num_distance.push_back(_distance(0.2, 3.3));  
  _num_distance.push_back(_distance(0.4, -1.6));
  _num_distance.push_back(_distance(1.9, -1.6));
  _num_distance.push_back(_distance(2.96, 2.28));
  _num_distance.push_back(_distance(-2.8, 0.6));
  _num_distance.push_back(_distance(4.8, 0.1));
  _num_distance.push_back(_distance(-0.6, -2.6));
  _num_distance.push_back(_distance(-3.45, 0.9));
  */
  _log.open(file_name);
  if (!_log) {
    cout << "ArdrnineTf cannot open file to log" << endl;
  }

  _ref_trans = this->get_transform("odom", "ardrone_base_link");
  tf::Transform input(_ref_trans.getRotation(), _ref_trans.getOrigin());
  _br.NewBr(input, "odom", "ref_pose");
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
  //ref_qua.setEulerZYX(yaw_plane - angle_offset, 0.0, 0.0);
  ref_qua.setEulerZYX(yaw_plane - angle_offset, pitch, roll); 

  tf::Transform input(ref_qua, _ref_trans.getOrigin());
  _ref_trans.setData(input);
  _br.SetRefPose(input, "odom", "ref_pose");

  //Log Info
  LogCurTime();
  _log << "set number" << _cur_number << "'s referace pose" << endl;
  _ref_trans.getBasis().getEulerYPR(yaw_plane, pitch, roll);
  _log << "Tx = " << _ref_trans.getOrigin().x()
       << " Ty = " << _ref_trans.getOrigin().y()
       << " Tz = " << _ref_trans.getOrigin().z() << endl;

  _log << "yaw = " << yaw_plane << " pitch = " 
       << pitch << " roll = " << roll << endl;

  _log << "angle_offset = " << angle_offset << endl;

}

void ArdroneTf::GetDiff(double& error_x, double& error_y, 
    double& error_turn) {

  tf::StampedTransform ref_to_base;
  double yaw, yaw_ref, pitch, roll;
  ref_to_base = get_transform("ref_pose", "ardrone_base_link");
  error_x = -ref_to_base.getOrigin().x() - 
      _num_distance[_cur_number][_tar_number]._x;

  error_y = -ref_to_base.getOrigin().y() - 
      _num_distance[_cur_number][_tar_number]._y;

  // Two ways to calculate error_turn
  ref_to_base.getBasis().getEulerYPR(error_turn, pitch, roll);
  error_turn = -error_turn;

  /*
  this->get_transform("odom", "ardrone_base_link").getBasis().getEulerYPR(
      yaw, pitch, roll);

  _ref_trans.getBasis().getEulerYPR(yaw_ref, pitch, roll);
  //mat.getEulerYPR(yaw_ref, pitch, roll);
  error_turn = yaw - yaw_ref;
  if (yaw_diff < -PI) {
    error_turn += 2 * PI;
  }
  else if (yaw_diff > PI) {
    error_turn -= 2 * PI;
  }
  */

  //Log Info
  LogCurTime();
  _log << "error_x = " << error_x << "  error_y = " << error_y 
      << "  error_turn = " << error_turn <<endl;

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
      - _num_distance[_cur_number][_tar_number]._x;

}

double ArdroneTf::YDiff() {
  return - this->get_transform("ref_pose", "ardrone_base_link").getOrigin().y() 
      - _num_distance[_cur_number][_tar_number]._y;

}

//void ArdroneTf::SetRefQuaternion() {
//  _ground_quaternion = 
//      this->get_transform("odom", "ardrone_base_link").getRotation();
//
//}

//int ArdroneTf::get_cur_number() {
//  return _cur_number;
//}

void ArdroneTf::LogCurTime() {
  time_t timep;
  struct tm *a;
  time(&timep);
  a = localtime(&timep);
  _log << endl << a->tm_mday << " " << a->tm_hour << ":"
       << a->tm_min << ":" << a->tm_sec << endl;

}
