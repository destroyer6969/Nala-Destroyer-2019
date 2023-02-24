#ifndef RAISETHEFLAG_H
#define RAISETHEFLAG_H

#include "misi.h"
#include <QString>
#include <common/mavlink.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


class RaiseTheFlag : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT1,
        WAYPOINT2,
        SETPOINT,
        STOPPOINT
    };
    enum {
      DANAU_8           = 0,
      REED_CANAL_PARK   = 1,
    };

public:
  RaiseTheFlag();
  ~RaiseTheFlag();
  int speed2;
  int sudut;
  int dock;
  QString statusMission;

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

signals:
  void sendFlag(int);
  void sendStatusMission(QString);
  void launchDrone();
  void stopDrone();
  void switchCam(int);

public slots:
  void setSpeed2(int);
  void setSudut(int);
  void setDroneWaypoint(unsigned, const PositionType&);
  void deleteDroneWaypoint(int);
  void droneCamOn();
  void droneCamOff();
  void stop();


private:
  void init();
  void setNextDest();
  void getDockPosition(double);
  int controlCircleWaypoint(double, bool=1);
  void anotherSRFAvoidance();
  void initDroneWaypoints();
  double dockLat, dockLon;
  double longitudeConst[2];
  double latitudeConst;
  double circleRadius;
  double radToDegrees;
  int location;
  int sevenSegmentCounter[4];
  QThread *droneThread;
};

#endif // MISI_H
