#ifndef FINDTHEPATH_H
#define FINDTHEPATH_H

#include "misi.h"
#include <QString>



class FindThePath : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        STOPPOINT
    };

public:
  FindThePath();
  ~FindThePath();
  int mode;

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved


public slots:
    void setMode(int);

private:
    QMutex myMtx;
    void init();
    void setNextDest();
    bool equalBox(object_detection::BoundingBox&, object_detection::BoundingBox&);
    bool anotherSRFAvoidance();
    int srfThresholdJauh, srfThresholdDeket;
};

#endif // MISI_H
