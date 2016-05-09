/*
* CMDReciever.h
*
*  Created on: May 9, 2016
*      Author: Ljw
*/

#ifndef CMDRECIEVER_H_
#define CMDRECIEVER_H_

#include "pthread.h"

#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;

enum ModeType {TAKEOFF, LAND, STOP, FLYING}

class CMDReciever {
private:
	ModeType _mode;

	pthread_mutex_t _mode_mutex;

	const char* _filePath;
	ofstream log;

public:
	CMDReciever(const char* fileName);
	virtual ~CMDReciever();

	void SaveImage(const cv::Mat &src);
	ModeType GetMode();
	void SetMode(ModeType mode);
};

#endif /* CMDReciever_H_ */
