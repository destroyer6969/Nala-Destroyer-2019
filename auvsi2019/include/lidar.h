#ifndef LIDAR_H
#define LIDAR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rplidar.h"
#include <QObject>
#include <QImage>
#include <QPixmap>
#include <opencv2/opencv.hpp>
#include <qdebug.h>
#include <misi.h>
#include <QPainter>
#include <string.h>
#include <string>
#include <cmath>
#include <iostream>

#define PI_ 3.14159265
#define tosca 0,232,181
#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

/*
    each 0.000001 degree latitude = 11.12 cm
   Danau 8:
    each 0.000001 degree longitude = 11.03 cm
   Reed Canal Park:
    each 0.000001 degree longitude = 9.711 cm
*/

//  Scan Mode: Standard, Express, Boost, Sensitivity, Stability

enum {
  DANAU_8           = 0,
  REED_CANAL_PARK   = 1,
};
class Lidar : public QObject{
    Q_OBJECT
public:
    Lidar(QObject *parent = Q_NULLPTR);
    ~Lidar();
    void initImageMap();
    bool getRPLIDARDeviceInfo(RPlidarDriver * drv);
    bool checkRPLIDARHealth(RPlidarDriver * drv);
    void stopMotor();
    void startMotor();
    bool isRunning();
    int getPortNum();
signals:
    void sendImage(QPixmap*);
    void requestImage();
    void obstacle(double, double);
    void leftObstacle(double);

public slots:
    void run(std::vector<std::string>*);
    void getCurrentPos(double, double, double);
    void getLocation(double, double, double, double);
    void getImageMap(QPixmap*);
    void stop();

private:
    bool running;
    PositionType currentPos;
    PositionType obstaclePos;
    QPoint currentPoint;
    QPoint obstaclePoint;
    double compass;
    double range;
    double angle;
    double angleFromBoat;
    double latDiff;
    double longDiff;
    double longitudeConst[2];
    double latitudeConst;
    double latitude_awal;
    double longitude_awal;
    double skala_latitude;
    double skala_longitude;
    double xRange, yRange;
    double nearestXRange, nearestYRange, nearestLeftObstacle;
    RPlidarDriver *drv;
    bool inverted;
    int block;
    int portNum;
    double miring;
    double maxDistance;
    bool nhCreated;
    bool angleCompensate;
    int angleCompensateMultiple;
    ros::NodeHandle *nh;
    std::string scanMode;
    int location;
    double radToDegrees;
    u_result op_result;
    bool lidarConnected;
    float getAngle(const rplidar_response_measurement_node_hq_t& node);
    bool connectLidar(std::vector<std::string>*);
    QPoint calculatePoint(const PositionType&);
    QPixmap *map;
};

#endif // LIDAR_H