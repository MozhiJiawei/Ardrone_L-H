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

using namespace std;

class ArdroneTf {
public:
  ArdroneTf(const char* file_name);
  virtual ~ArdroneTf();

  void SetRefPose(int number);
  double YawDiff();
  double XDiff();
  double YDiff();

private:
  struct _distance{
    double _x;
    double _y;

    _distance(double x, double y) : _x(x), _y(y) {}
  };
  vector<_distance> _num_distance;

  tf::TransformListener _listener;
  tf::TransformBroadcaster _broadcaster;

  const char* _file_path;
  int _cur_number;

  tf::StampedTransform get_transform(const char* frame1, const char* frame2);
};

#endif // !ARDRONETF_H_

