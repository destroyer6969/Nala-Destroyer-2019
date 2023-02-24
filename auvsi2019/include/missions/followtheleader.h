#ifndef FOLLOWTHELEADER_H
#define FOLLOWTHELEADER_H

#include "misi.h"
#include <QString>

#define RED_FLAG    1
#define BLUE_FLAG   2


class FollowTheLeader : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        SETPOINT,
        STOPPOINT
    };

public:
  FollowTheLeader();
  ~FollowTheLeader();
  int speed2;
  int flagColor;

  QString statusMission;

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

signals:
  void sendFlagColor(int);
  void sendStatusMission(QString);

public slots:


private:

  void init();
  void setNextDest();
};

#endif // MISI_H
