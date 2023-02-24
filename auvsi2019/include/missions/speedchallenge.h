#ifndef SPEEDCHALLENGE_H
#define SPEEDCHALLENGE_H

#include "misi.h"
#include <QString>


class SpeedChallenge : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        STOPPOINT
    };


public:
  SpeedChallenge();
  ~SpeedChallenge();

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

public slots:

private:
  void init();
  void setNextDest();
  constanta pidTmp;
};

#endif // MISI_H
