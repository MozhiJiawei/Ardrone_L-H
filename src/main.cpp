/*
 * main.cpp
 *
 *  Created on: May 14, 2015
 *      Author: mrh
 */

#include <iostream>
#include <iomanip>
#include "ros/ros.h"
#include "math/SL_Matrix.h"
#include "fstream"
#include "std_msgs/String.h"

#include "imgproc/SL_Image.h"
#include "imgproc/SL_ImageIO.h"

#include "tools/GUI_ImageViewer.h"
#include "tools/SL_Print.h"
#include "tools/SL_DrawCorners.h"

#include "ROSThread.h"
#include "VideoRecorder.h"
#include "IMURecorder.h"
#include "ImgRecon.h"
#include "GridDetector.h"
#include "PIDController.h"
#include "AffineTransform.h"
#include "PredictNumber.h"
#include "SearchNumber.h"
#include "CMDReciever.h"
#include "time.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include "opencv2/legacy/blobtrack.hpp"
#include "math.h"
#define PI 3.14159265

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <tf/transform_listener.h>
#include <tf/transform_broacaster.h>
using namespace std;

#define Test 1

static int mGrids = 5;
static int nGrids = 6;
static int setcamera = 0;

void writeFeatPts(const vector<cv::Point2f>& featPts, const char* filePath) {
  ofstream file(filePath);
  if (!file)
    repErr("cannot open '%s' to write!", filePath);

  for (size_t i = 0; i < featPts.size(); i++) {
    file << featPts[i].x << " " << featPts[i].y << endl;
  }
}

void writeIMUMeas(const vector<IMUData>& imumeas, const char* filePath) {
  std::ofstream file(filePath);
  if (!file)
    repErr("cannot open '%' to write!", filePath);

  for (size_t i = 0; i < imumeas.size(); i++) {
    imumeas[i].write(file);
  }
  file.close();
}

#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
ofstream fout("/home/mozhi/Record/test.txt", ios::app);
#include "ARDrone.h"

void* Control_loop(void* param) {
  ARDrone drone;
  drone.setup();
  CMDReciever cmdreader("/home/mozhi/Logs/cmd.txt", drone);
  IMURecorder imureader("/home/mozhi/Record/imu.txt");
  VideoRecorder videoreader("/home/mozhi/Record/video_ts.txt", "/home/mozhi/Record/video.avi");
  ROSThread thread(imureader, videoreader, cmdreader);
  thread.showVideo = true;
  ros::Rate loop_rate(50);
  ////////////////////////////////
  ImgRecon img_recon(NULL);
  IplImage *imgsrc;
  CvSize imgSize = { 640,360 };
  imgsrc = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);

  Mat imgmat;

  system("rosservice call /ardrone/setcamchannel 1");
  //system("rosservice call /ardrone/setrecord 1");
  ///////////////////////// PID control parameters
  double targetx, targety;
  double centerx, centery;
  double errorx, errory, errorturn;
  double lasterrorx, lasterrory, lasterrorturn;
  double targetvx, targetvy, targetv;
  int targeth, targethland = 700;
  double leftr = 0, forwardb = 0, upd = 0, turnleftr = 0;
  double vk = 2.0;//0.001;
  static double kp = 4.0;//0.0001;
  static double kd = 150.0;
  static double ki = 0.0;
  double scale = 3;
  static double vkp = 5, vkd = 20, vki = 0;
  ///////////////////////////////////PID调节的初始量///////////////////////////////
  PIDController pidX, pidY, pidZ;
  PIDController pidVX, pidVY, pidVZ, PidW;
  pidX.setParam(kp, ki, kd, 2);
  pidY.setParam(kp, ki, kd, 2);
  pidZ.setParam(kp, ki, kd, 3);
  pidVX.setParam(vkp, vki, vkd, 2);
  pidVY.setParam(vkp, vki, vkd, 2);
  pidVZ.setParam(vkp, vki, vkd, 2);
  PidW.setParam(vkp, 20, vkd, 2);
  targetx = 320, targety = 185;
  //////////////////////////////////////////////////////////
  ofstream log;
  char filename[50];
  time_t timep;
  struct tm *a;
  time(&timep);
  a = localtime(&timep);
  sprintf(filename, "/home/mozhi/Logs/%02d_%02d_%02d_%02d.txt",
    a->tm_mday, a->tm_hour, a->tm_min, a->tm_sec);

  log.open(filename);
  if (!log) {
    cout << "cannot open file to log" << endl;
  }
  cout << "Start!" << endl;
  ///////////////////////////////////////////////////////////
  ModeType cur_mode = STOP, next_mode = STOP;
  int frame_count = 0, lostframe = 0;
  ///////////////////////////////////////////////////////////
  double takeoff_time;
  clock_t pid_stable_time;
  clock_t landing_time;
  char c;
#if Test
  tf::TransformListener listener;
  while (ros::ok()) {
    usleep(250000);
    tf::StampedTransform odom_to_link;
    try {
      listener.lookupTransform("odom", "ardrone_base_link",
        ros::Time(0), odom_to_link);

    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    time(&timep);
    a = localtime(&timep);
    log << endl << a->tm_mday << " " << a->tm_hour << ":"
      << a->tm_min << ":" << a->tm_sec << endl;

    log << "form lister: " << (double)odom_to_link.stamp_.toSec() << endl;
    log << "Tx = " << odom_to_link.getOrigin().x()
      << " Ty = " << odom_to_link.getOrigin().y()
      << " Tz = " << odom_to_link.getOrigin().z() << endl;

    log << "Qx = " << odom_to_link.getRotation().x()
      << " Qy = " << odom_to_link.getRotation().y()
      << " Qw = " << odom_to_link.getRotation().z()
      << " Qw = " << odom_to_link.getRotation().w() << endl;

    log << "form Odo subsrcriber: " 
      << thread.odometry.header.stamp.sec << endl;

    log << "Tx = " << thread._odometry.pose.pose.position.x
      << " Ty = " << thread._odometry.pose.pose.position.y
      << " Tz = " << thread._odometry.pose.pose.position.z << endl;

    log << "Qx = " << thread._odometry.pose.pose.orientation.x
      << " Qy = " << thread._odometry.pose.pose.orientation.y
      << " Qw = " << thread._odometry.pose.pose.orientation.z
      << " Qw = " << thread._odometry.pose.pose.orientation.w << endl;
  }
  ros::spin();
  double control_time;
  double current;
  drone.takeOff();
  cout << "take off" << endl;
  control_time = (double)ros::Time::now().toSec();
  while ((double)ros::Time::now().toSec() < control_time + 6);
  drone.move(0.0, 0.1, 0.0, 0.0);
  cout << "move 0.05" << endl;
  while ((double)ros::Time::now().toSec() < control_time + 8) {
    usleep(250000);
    log << "move 0.1   " << "  vx: "
        << setw(8) << thread.navdata.vx << " vy:" << setw(8)
        << thread.navdata.vy << endl;//<<" "<< thread.navdata.vz;

  }
  drone.move(0.0, 0.2, 0.0, 0.0);
  cout << "move 0.2" << endl;
  while ((double)ros::Time::now().toSec() < control_time + 10) {
    usleep(250000);
    log << "move 0.2  " << "  vx: "
        << setw(8) << thread.navdata.vx << " vy:" << setw(8)
        << thread.navdata.vy << endl;//<<" "<< thread.navdata.vz;
  
  }
  drone.move(0.0, 0.1, 0.0, 0.0);
  cout << "move 0" << endl;
  while ((double)ros::Time::now().toSec() < control_time + 12) {
    usleep(250000);
    log << "move 0.1   " << "  vx: "
        << setw(8) << thread.navdata.vx << " vy:" << setw(8)
        << thread.navdata.vy << endl;//<<" "<< thread.navdata.vz;

  }
  drone.land();
#endif
  
  cvNamedWindow("a", 1);
  while (ros::ok()) {
    usleep(1000);
    lostframe++;
    if (lostframe > 100) drone.hover(); // if the video is not fluent
    if (lostframe > 3000) drone.land(); // if the video is not fluent
    //////////////////////////test/////////////////////////////////////////////////
    if (videoreader.newframe) {
      cur_mode = cmdreader.GetMode();
      if (cur_mode == START) {
        cout << "START";
      }
      frame_count++;
      lostframe = 0;
      videoreader.newframe = false;
      cout << "Battery:" << thread.navdata.batteryPercent << endl;

      videoreader.getImage(imgmat);
      *imgsrc = imgmat;
      img_recon.ReInit(imgsrc);
      switch (cur_mode) {
      case START:
        drone.hover();
        drone.takeOff();
        takeoff_time = (double)ros::Time::now().toSec();
        while((double)ros::Time::now().toSec() < takeoff_time + 4);
        next_mode = TAKEOFF;
        break;
      case TAKEOFF:
        time(&timep);
        a = localtime(&timep);
        log << endl << a->tm_mday << " " << a->tm_hour << ":"
            << a->tm_min << ":" << a->tm_sec << endl;

        if (img_recon.ContourExist()) {
          cout << "conter found" << endl;
          centerx = img_recon.GetCenterPoint().x;
          centery = img_recon.GetCenterPoint().y;
          log << "center founded: x = " << centerx
            << "  y = " << centery << endl;

          CLIP3(10.0, centerx, 590.0);
          CLIP3(10.0, centery, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          errorx = centerx - targetx;
          errory = centery - targety;
          targetvx = -vk * errory - vk*(errory - lasterrory);
          targetvy = -vk * errorx - vk*(errorx - lasterrorx);
          if (errory > 80 || errory < -80) {
            targetvx += -vk * errory + 80 * vk;
          }
          if (errorx > 80 || errorx < -80) {
            targetvy += -vk * errorx + 80 * vk;
          }
          CLIP3(-1500.0, targetvx, 1500.0);
          CLIP3(-1500.0, targetvy, 1500.0);

          forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
          leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
          leftr /= 15000;        forwardb /= 15000;
          if (thread.navdata.altd < 1400) {
            upd = 0.002 * (1400 - thread.navdata.altd);
          }
          else if (thread.navdata.altd > 1450) {
            upd = 0.002 * (1450 - thread.navdata.altd);
          }
          else {
            upd = 0;
          }
          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          CLIP3(-0.2, upd, 0.2);
          turnleftr = 0;

          if (abs(errorx) < 15 && abs(errory) < 15) {
            if (upd == 0) {
              if ((clock() - pid_stable_time) / CLOCKS_PER_SEC * 1000
                  > 500) {

                log << "TAKEOFF Complete! Start Landing" << endl;
                next_mode = LAND;
              }
              log << "PID Complete" << endl;
            }
          }
          else {
            pid_stable_time = clock();
            log << "e_x = " << errorx << "  e_y = " << errory << endl;
            log << "t_vx = " << targetvx << "  t_vy = " << targetvy << endl;
            log << " Height:" << setw(8) << thread.navdata.altd << "  vx: "
              << setw(8) << thread.navdata.vx << " vy:" << setw(8)
              << thread.navdata.vy << endl;//<<" "<< thread.navdata.vz;

            log  << "  x_forward = " << forwardb << "y_left = " << leftr
              << "  z_up = " << upd << endl;

          }
        }
        else {
          if (thread.navdata.altd < 1400) {
            upd = 0.002 * (1400 - thread.navdata.altd);
          }
          else if (thread.navdata.altd > 1450) {
            upd = 0.002 * (1450 - thread.navdata.altd);
          }
          else {
            upd = 0;
          }
          CLIP3(-0.2, upd, 0.2);
          log << "cannot find conter, keep rising" << endl;
        }
        break;
      case LAND: 
        if (img_recon.ContourExist()) {
          centerx = img_recon.GetCenterPoint().x;
          centery = img_recon.GetCenterPoint().y;

          CLIP3(10.0, centerx, 590.0);
          CLIP3(10.0, centery, 350.0);
          lasterrorx = errorx;
          lasterrory = errory;
          errorx = centerx - targetx;
          errory = centery - targety;
          targetvx = -vk * errory - vk*(errory - lasterrory);
          targetvy = -vk * errorx - vk*(errorx - lasterrorx);
          if (errory > 80 || errory < -80) {
            targetvx += -vk * errory + 80 * vk;
          }
          if (errorx > 80 || errorx < -80) {
            targetvy += -vk * errorx + 80 * vk;
          }
          CLIP3(-1500.0, targetvx, 1500.0);
          CLIP3(-1500.0, targetvy, 1500.0);
          forwardb = pidVX.getOutput(targetvx - thread.navdata.vx, 0.5);
          leftr = pidVY.getOutput(targetvy - thread.navdata.vy, 0.5);
          leftr /= 15000;        forwardb /= 15000;
          if (thread.navdata.altd < 350) {
            upd = 0.002 * (350 - thread.navdata.altd);
          }
          else if (thread.navdata.altd > 400) {
            upd = 0.002 * (400 - thread.navdata.altd);
          }
          else {
            upd = 0;
          }
          CLIP3(-0.1, leftr, 0.1);
          CLIP3(-0.1, forwardb, 0.1);
          CLIP3(-0.2, upd, 0.2);
          turnleftr = 0;
          if (upd == 0 && abs(errorx) < 15 && abs(errory) < 15) {
            drone.hover();
            drone.land();
            landing_time = clock();
            while (static_cast<double>(clock() - landing_time)
              / CLOCKS_PER_SEC * 1000 > 6000);

            next_mode = START;
          }
        }
        break;
      default:
        break;
      }
    }

    if (cur_mode == cmdreader.GetMode() && cur_mode != MANUL) {
      cmdreader.RunNextMode(next_mode, leftr, forwardb, upd,
        turnleftr);

    }
    if (cmdreader.GetMode() == MANUL) {
      if (static_cast<double>(clock() - cmdreader.GetManualTime()) 
          / CLOCKS_PER_SEC * 1000 > 1000) {

        drone.hover();
      }
    }
    cvShowImage("a", imgsrc);
    cv::waitKey(1);
  }
  drone.land();
  cvReleaseImage(&imgsrc);
  return 0;
}

void ROSControl_main(int argc, char** argv) {
  //ros::init(argc, argv, "listener");
  pthread_t ROS_thread, control_thread;
  int rc = pthread_create(&control_thread, NULL, Control_loop, 0);
  if (rc) {
    printf("ERROR; return code from pthread_create() is %d\n", rc);
    exit(-1);
  }

  pthread_join(control_thread, NULL);
}

CvANN_MLP bp;
#include "IMUVideoSync.h"
int main(int argc, char** argv)
{
#if premode==1
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel1.xml");
#else
#if premode==2
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel2.xml");
#else
  bp.load("/home/mozhi/catkin_ws/src/Ardrone_L-H/src/NumberTrain/bpModel_op.xml");
#endif
#endif
  ros::init(argc, argv, "ARDrone_test");
  ROSControl_main(argc, argv);
  fout.close();

  return 0;
}
