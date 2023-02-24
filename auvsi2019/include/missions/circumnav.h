#ifndef CIRCUMNAV_H
#define CIRCUMNAV_H

#include "misi.h"
#include <QString>



class CircumNav : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        BUOY1,
        BUOY2,
        BUOY3,
        BUOY4
    };

public:
  CircumNav();
  ~CircumNav();


  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved


public slots:


private:
  void init();
  void setNextDest();
};

#endif // MISI_H
