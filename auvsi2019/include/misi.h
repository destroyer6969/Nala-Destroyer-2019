#ifndef MISI_H
#define MISI_H

#include <ros/ros.h>
#include <object_detection/ObjectAttr.h>
#include <QObject>
#include <vector>
#include <qdebug.h>
#include <QString>
#include <cmath>
#include <QMutex>
#include <QJsonObject>
#include <QJsonArray>
#include <unordered_map>
#include <iostream>
#include <QTimer>
#include <QPoint>
#include <time.h>
#include <exception>
#include <string>
#include <algorithm>
#include <string.h>
#include "drone.h"

using namespace std;

#define PI               3.14159265
#define NULLPOS    -1000.0  // absence of position
#define NUM_CLASSES 8

/*
    each 0.000001 degree latitude = 11.12 cm
   Danau 8:
    each 0.000001 degree longitude = 11.03 cm
   Reed Canal Park:
    each 0.000001 degree longitude = 9.711 cm
*/

enum {
  BALL              = 0,
  RED_CAN_BUOY      = 1,
  GREEN_CAN_BUOY    = 2,
  ANOTHER_CAN_BUOY  = 3,
  NUMBER1           = 4,
  NUMBER2           = 5,
  NUMBER3           = 6,
  NUMBER4           = 7,
  SEVEN_SEGMENT1    = 8,
  SEVEN_SEGMENT2    = 9,
  SEVEN_SEGMENT3    = 10,
  SEVEN_SEGMENT4    = 11,
};

typedef struct{
  double latitude;
  double longitude;
  bool isNull(){
    if (latitude==NULLPOS || longitude==NULLPOS){
      return true;
    }
    else return false;
  }
} PositionType;

typedef struct{
  double p,i,d;
}constanta;

typedef struct{
  int yMin, xMin, xMax, yMax;
}boundingBox;

//////////////////////
/// \brief The Misi class
/// ABSTRACT class of missions
/// Can NOT be instantiated

class Misi : public QObject
{
    Q_OBJECT
public:
  Misi();
  int speed;
  int speedSlow;
  int travelingSpeed;
  unsigned NUM_WP;
  PositionType currentPos, startPos;
  double compass;
  std::string srfMode;
  Drone *drone;

  virtual void init() = 0;
  virtual int startMission() = 0;
  void setWaypoint(unsigned, const PositionType&);
  virtual void setDroneWaypoint(unsigned, const PositionType&);
  virtual void deleteDroneWaypoint(int);
  PositionType& getWaypoint(unsigned index);
  void clearWaypoints();
  void deleteWaypoint(int);
  QString getName();
  bool isRunning();
  void setObjAttr(const object_detection::ObjectAttr &msg);
  int getNumWaypoints();
  void setControlLine(int, int, int, int, int, int, int, int);
  void resetCameraControlPoint();
  QPoint getCameraControlPoint();
  void setSRF(int, int, int, int, int);
  int getControlLine(int);
  int getYThreshold();
  char* getControl();

  virtual void write(QJsonObject &obj) const = 0;       // write mission's attribute to JSON to be saved
  void writeCommons(QJsonObject &obj) const;
  void setLidarObstacle(double, double);
  void setLidarLeftObstacle(double);
protected:
  QMutex mtx_pr;
  vector<PositionType> waypoints;
  constanta pid[4];
  constanta pidDist;
  int num_pid;
  int nextMissionIndex;
  int active;
  int wpIdx;
  int lastWpIdx;
  int outServo;
  int outSRFAvoidance;
  int servoKamera;
  int topLeftX, topLeftY, botLeftX, botLeftY, topRightX, topRightY, botRightX, botRightY;
  int yThreshold;
  int servoMuterMax;
  int location;
  int controlDistanceOut;
  int lidarControlRange;
  int srfAvoidanceCounter[3];
  int speedReverse;
  char qweqwe;
  double sudutTujuan;
  double error;
  double errorSudut;
  double errorSebelum;
  double errorDist, errorDistSebelum;
  double jarak;
  double arriveThreshold;
  double xRangeLidar;
  double yRangeLidar;
  double lidarLeftObstacle;
  double startAngle, startAngle1, startAngle2;
  int srfL, srfR, srfML, srfMR, srfS;
  int useSomething, useSomething2;
  const double radToDegrees = PI/180.0;
  PositionType lastWaypoint;
  object_detection::ObjectAttr detectedObject;
  QString name;
  int squaredPixelDistance(int, int, int, int);
  int controlWaypoint(int, bool=1, int=0, bool=1, bool=0);
  int controlBetween2Object(int pidIdx, int x1, int x2);
  int controlBoatSide(int, int, int, int, int, int, int);
  int controlLidarAvoidance(int, double);
  int controlRange(int, double, double, bool=0);
  int srfAvoidance();
  double distanceLineToPoint(PositionType, PositionType, PositionType);
  void mirrorControlLine(int=0);
  void start();
  object_detection::BoundingBox getNearestObject(int, int=-1); // get nearest object from specific class
  object_detection::BoundingBox getNearestObject(); // get nearest object from all classes
  object_detection::BoundingBox getNearestObstacle(int, int, int=-1); // get nearest obstacle from specific class
  object_detection::BoundingBox getNearestObstacle(int); // get nearest obstacle from all classes
  virtual void setNextDest() = 0;
  bool running;
  bool newImage;
  QMutex mtx, mtx1, mtx2, mtx3, mtx4, mtx5, mtx6, mtx8, mtx9, mtx10;
  QPoint controlPoint;
  time_t startTime;
  char control[25];

  // break  conditions
  double timeout;
  int breakWPIndex;
  int useCamera;

  ros::NodeHandle n;
  ros::Subscriber objAttr_sub;

signals:
  void sendOutput(int, int, int, int, int);
  void arrived(int nextDest, const PositionType& pos);
  void updateSudutJarak(double, double, double);
  void setNewWaypoint(double, double, int);
  void signalDeleteWaypoint(int);
  void displayCurrentState(QString);

public slots:
  void setSpeed(int);
  void setSpeedSlow(int);
  void setP1(double);
  void setI1(double);
  void setD1(double);
  void setP2(double);
  void setI2(double);
  void setD2(double);
  void setP3(double);
  void setI3(double);
  void setD3(double);
  void setP4(double);
  void setI4(double);
  void setD4(double);
  void setPD(double);
  void setID(double);
  void setDD(double);
  void setNextMission(int);
  void setStatus(int);
  virtual void stop();
  void setLidarControlRange(int);
  void setYThreshold(int);
  void setServoMuterMax(int);
  void setUseSomething(int);
  void setUseSomething2(int);

  // slots for break conditions
  void setTimeout(double);
  void setBreakWPIndex(int);
  void setUseCamera(int);
};

#endif // MISI_H
