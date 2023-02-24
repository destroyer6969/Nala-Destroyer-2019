#ifndef MISSIONWRAPPER_H
#define MISSIONWRAPPER_H

#include <QObject>
#include <qdebug.h>
#include <QString>
#include <QMutex>
#include <QThread>
#include <QTimer>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <QPoint>
#include <object_detection/ObjectAttr.h>
#include "misi.h"
#include "missions/autodocking.h"
#include "missions/autonav.h"
#include "missions/circumnav.h"
#include "missions/findthepath.h"
#include "missions/followtheleader.h"
#include "missions/maintainheading.h"
#include "missions/raisetheflag.h"
#include "missions/returnhome.h"
#include "missions/slalom.h"
#include "missions/speedchallenge.h"

#define NUM_MISI 11
#define BAUDRATE 115200

enum {
    MISI_NULL           = 0,
    MISI_AUTONAV        = 1,
    MISI_SPEEDCHAL      = 2,
    MISI_AUTODOCK       = 3,
    MISI_RAISETHEFLAG   = 4,
    MISI_FINDPATH       = 5,
    MISI_FOLLOWLEADER   = 6,
    MISI_RETURNHOME     = 7,
    MISI_CIRCUMNAV      = 8,
    MISI_MHEADING       = 9,
    MISI_SLALOM         = 10,
};
class MissionWrapper : public QObject
{
    Q_OBJECT
public:
    MissionWrapper(QObject *parent = Q_NULLPTR);
    ~MissionWrapper();
    Misi* misi[NUM_MISI];
    unsigned missionState;
    const PositionType& getWaypoint(unsigned, unsigned);
    void changeMissionSequence(QString);
    // void clearWaypoints();
    void write(QJsonObject &obj) const;     // write missions' attributes to JSON to be saved
    void setControlLine(int, int, int, int, int, int, int, int);
    QPoint getCameraControlPoint();
    void resetCameraControlPoint();
    bool isOpened();
    int getPortNum();
    char* getOutput();
    char* getMissionControl();
    
private:
  PositionType currentPos;
  int misiIdx[11]={0,1,2,3,4,5,6,7,8,9,10};
  int it = 1;
  int forceStopped;
  int normalServoMotor, normalServoKamera;
  int fd;
  int motorKananTest, motorKiriTest, servoKananTest, servoKiriTest, servoKameraTest;
  bool runningTest;
  bool opened=0;
  bool panicStop;
  QMutex mtx, mtx_, rMtx;
  int init_serial_port(const char*);
  int portNum;
  int result;
  char outBuffer[30], inBuffer[5];
  int sysMode;
  int readCounter=0;
  time_t startTime;
signals:
  void nextMission(int misiIdx);
  void nextDestChanged(int, const PositionType*);
  void sendSudutJarak(double, double, double);
  void updateSudutJarak(double sudut, double error, double jarak);
  void clearOutputs();
  void displayCurrentMission(int);
  void switchMissionTab(int);
  void endMission();
  void sendControlLine(bool, int, int, int, int);
  void sendYThreshold(int);
  void sendSRFMode(std::string&);
  void newSysMode(int);
  
private slots:
  void getSudutJarak(double sudut, double error, double jarak);
  void onArrived(int nextDest, const PositionType& pos);
  void setWaypoint(unsigned misiIndex, unsigned index, const PositionType& pos);
  void getCurrentPos(double latitude, double longitude, double compass);
  void sendToSTM(int, int, int, int, int);
  void getLidarObstacle(double, double);

public slots:
  void run();
  void runTest();
  void stopTest();
  void forceStop(bool);
  void getDetectedObject(const object_detection::ObjectAttr& msg);
  void nextMission();
  void setMotorKananTest(int);
  void setMotorKiriTest(int);
  void setServoKananTest(int);
  void setServoKiriTest(int);
  void setServoKameraTest(int);
  void srfDataToMission(int, int, int, int, int);
  void lidarLeftObstacle(double);
  void connectToSTM(std::vector<std::string>*);
  void setTengahServoMotor(int);
  void setTengahServoKamera(int);
  void panic();
  void readFromSTM();
};

#endif // MISSIONWRAPPER_H
