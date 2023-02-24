#ifndef MAINTAINHEADING_H
#define MAINTAINHEADING_H

#include "misi.h"
#include <QString>


class MaintainHeading : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        STOPPOINT
    };

public:
  MaintainHeading();
  ~MaintainHeading();
  int sudut;

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

public slots:
  void setSudut(int);

private:
  void init();
  void setNextDest();
};

#endif // MISI_H
