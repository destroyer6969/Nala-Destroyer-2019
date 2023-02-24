#ifndef RETURNHOME_H
#define RETURNHOME_H

#include "misi.h"
#include <QString>


class ReturnHome : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        STOPPOINT
    };

public:
  ReturnHome();
  ~ReturnHome();


  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

public slots:


private:
  void init();
  void setNextDest();
};

#endif // MISI_H
