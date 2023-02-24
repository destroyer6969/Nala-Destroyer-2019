#ifndef SLALOM_H
#define SLALOM_H

#include "misi.h"
#include <QString>


class Slalom : public Misi
{
    Q_OBJECT

    enum {
        WAYPOINT,
        STOPPOINT
    };

public:
  Slalom();
  ~Slalom();

  int startMission();
  void write(QJsonObject &obj) const;       // write mission's attribute to JSON to be saved

public slots:


private:
  void init();
  void setNextDest();
};

#endif // MISI_H
