/*
* CMDReciever.h
*
*  Created on: May 9, 2016
*      Author: Ljw
*/

#ifndef CMDRECIEVER_H_
#define CMDRECIEVER_H_

#include "pthread.h"
#include "ARDrone.h"
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;

enum ModeType {TAKEOFF, LAND, STOP, FLYING, MANUL, START};

class CMDReciever {
public:
	CMDReciever(const char* fileName);
	virtual ~CMDReciever();

	void SaveImage(const cv::Mat& src);
	ModeType GetMode();
	void RunNextMode(ModeType mode, double y_left, double x_forward,
      double z_up, double angle_turn, ARDrone& drone);

private:
	ModeType _mode;

	pthread_mutex_t _mode_mutex;

	const char* _filePath;
	ofstream log;
};

#endif /* CMDReciever_H_ */
