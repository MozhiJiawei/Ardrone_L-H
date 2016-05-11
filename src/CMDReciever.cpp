/*
* CMDReciever.cpp
*
*  Created on: May 16, 2013
*      Author: Ljw
*/

#include "CMDReciever.h"
#include "time.h"


CMDReciever::CMDReciever(const char* fileName) {
	_filePath = fileName;
	_mode = STOP;
	pthread_mutex_init(&_mode_mutex, 0);
}

CMDReciever::~CMDReciever() {
	pthread_mutex_lock(&_mode_mutex);
}

void CMDReciever::SaveImage(const cv::Mat& src) {
	char filename[50];
	time_t timep;
	struct tm *a;
	time(&timep);
	a = localtime(&timep);
	sprintf(filename, "/home/mozhi/Logs/%02d_%02d_%02d_%02d.bmp",
		a->tm_mday, a->tm_hour, a->tm_min, a->tm_sec);

	cout << "Save image to" << endl << filename << endl;
	cv::imwrite(filename, src);
}

ModeType CMDReciever::GetMode() {
	pthread_mutex_lock(&_mode_mutex);
	ModeType cur_mode = _mode;
	pthread_mutex_unlock(&_mode_mutex);
	cout << "get mode success";
	return cur_mode;
}

void CMDReciever::SetMode(ModeType mode) {
	pthread_mutex_lock(&_mode_mutex);
	_mode = mode;
	pthread_mutex_unlock(&_mode_mutex);
}

void CMDReciever::RunNextMode(ModeType mode, double y_left, double x_forward,
    double z_up, double angle_turn, ARDrone& drone) {

	pthread_mutex_lock(&_mode_mutex);
	_mode = mode;
	pthread_mutex_unlock(&_mode_mutex);
  cout << "running mode = " << mode << endl;

  switch (mode) {
  case TAKEOFF:
  case LAND:
  case FLYING:
    drone.move(y_left, x_forward, z_up, angle_turn);
    break;
  default:
    break;
  }
}
