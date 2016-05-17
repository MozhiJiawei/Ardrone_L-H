/*
* ArdroneTf.h
*
*  Created on: May 16, 2016
*      Author: Ljw
*/

#ifndef ARDRONETF_H_
#define ARDRONETF_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#define PI 3.14159265

using namespace std;

class ArdroneTf {
public:
  ArdroneTf(const char* file_name);
  virtual ~ArdroneTf();

  void SetRefPose();
  void SetRefQuaternion();
  double YawDiff();
  double XDiff();
  double YDiff();
  int get_cur_number();

private:
  struct _distance{
    double _x;
    double _y;

    _distance(double x, double y) : _x(x), _y(y) {}
  };
  vector<_distance> _num_distance;

  tf::TransformListener _listener;
  tf::TransformBroadcaster _broadcaster;
  tf::Quaternion _ground_quaternion;

  const char* _file_path;
  int _cur_number;

  tf::StampedTransform get_transform(const char* frame1, const char* frame2);
};

#endif // !ARDRONETF_H_
