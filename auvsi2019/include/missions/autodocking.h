#ifndef AUTODOCKING_H
#define AUTODOCKING_H

#include "misi.h"
#include <time.h>
#include <stdlib.h>
#include <QString>



class AutoDocking : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT1,
        WAYPOINT2,
        SETPOINT,
        STOPPOINT
    };

public:
  AutoDocking();
  ~AutoDocking();

  int speed2;
  int threshold;
  int sudut;
  int dock;
  int maxPinger;
  int frequency;
  int counter;
  int pinger;
  QString statusMission;

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

private:
  void setNextDest();
  
signals:
  void dockChanged(int);
  void counterChanged(int);
  void pingerChanged(int);
  void statusMissionChanged(QString);
  void sendDock(int);

public slots:
  //// SETTER ///////
  void setSpeed2(int);
  void setThreshold(int);
  void setSudut(int);
  void setMaxPinger(int);
  void setFrequency(int);


private:
  void init();
};

#endif // MISI_H
