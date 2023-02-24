#ifndef IMGRECEIVER_H
#define IMGRECEIVER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <object_detection/ObjectAttr.h>
#include <opencv2/opencv.hpp>
#include <QObject>
#include <QImage>
#include <vector>
#include "misi.h"
enum {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    NUM_COLORS
};

#define MAX_NUM_OBJECTS 20
#define MIN_OBJECT_AREA 20
#define MAX_OBJECT_AREA 800

using namespace cv;
using namespace std;

class ImgReceiver : public QObject
{
    Q_OBJECT
public:
  ImgReceiver(QObject *parent = Q_NULLPTR);
  ~ImgReceiver();

  ros::NodeHandle n;

  int isThreshold, currentColor;
  int hmin[NUM_COLORS], smin[NUM_COLORS], vmin[NUM_COLORS];
  int hmax[NUM_COLORS], smax[NUM_COLORS], vmax[NUM_COLORS];
  int erode[NUM_COLORS], dilate[NUM_COLORS];


public slots:
  void start();
  void stop();
  void kill();
  void switchCam(int);

  // thresholdings
  void colorChange(int);
  void hminChange(int);
  void sminChange(int);
  void vminChange(int);
  void hmaxChange(int);
  void smaxChange(int);
  void vmaxChange(int);
  void erodeChange(int);
  void dilateChange(int);
  void drawRectangles();

signals:
  void imgReceived(QImage*);
  void sendDetectedObject(const object_detection::ObjectAttr &msg);
  void sendBinaryImage(QImage* bin);

private:
  image_transport::ImageTransport itrans;
  image_transport::Subscriber img_sub;
  bool isKilled;
  ros::NodeHandle n2;
  ros::Subscriber objAttr_sub;
  ros::Publisher camSwitch_pub;
  std_msgs::Int8 camSwitch_msg;
  int camToSwitch;
  VideoCapture cap;
  Mat frame;
  object_detection::ObjectAttr msg;
  QMutex mtx_cam;

  void objAttrReceive(const object_detection::ObjectAttr &msg);
  void processImage(Mat &img);
//  void imgReceive(const sensor_msgs::ImageConstPtr& msg);
  QImage* matToQImage(const cv::Mat&);
  void thresholdImage(cv::Mat &img);
  void detectObjectByColor(cv::Mat *hsv, int idxColor);
  void morphOps(cv::Mat *bin, int idxColor);

};

#endif // IMGRECEIVER_H
