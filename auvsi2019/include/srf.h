#ifndef SRF_H
#define SRF_H

#include <QObject>
#include <qdebug.h>
#include <QString>
#include <QMutex>
#include <QThread>
#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <iostream>
#include <string>
#include <cmath>

class SRF : public QObject
{
    Q_OBJECT
public:
    SRF(QObject *parent = Q_NULLPTR);
    ~SRF();
    int* getSRFData();
    bool isOpened();
    int getPortNum();
private:
    int init_serial_port(const char *);
    bool opened, changingMode;
    int fd, result;
    int srf[5];
    bool running;
    char srfDataStr[25];
    int srfData[5];
    int res;
    QMutex mtx, mtx1;
    time_t startTime;
    int portNum;
    const char *baudRate, *baudRate_;
signals:
    void sendData(int, int, int, int, int);
public slots:
    void run(std::vector<std::string>*);
    void stop();
    void setSRFMode(std::string &);
};

#endif // MISSIONWRAPPER_H
