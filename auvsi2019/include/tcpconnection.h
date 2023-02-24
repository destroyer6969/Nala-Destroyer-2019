#ifndef TCPCONNECTION_H
#define TCPCONNECTION_H

#include <qdebug.h>
#include <QString>
#include <QTimer>
#include <string.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <time.h>
#include <QThread>
#include <QMutex>

class TCPConnection : public QObject
{
    Q_OBJECT
public:
    TCPConnection(QObject *parent = Q_NULLPTR);
    ~TCPConnection();
    QTimer *timerHeartBeat;


public slots:
    void openConnection(const char*, int);
    void startConnection();
    void stopConnection();
    void closeConnection();
    void getCurrentPos(double, double);
    void setCourse(int);
    void setMission(int);
    void setSysMode(int);

private:
    struct sockaddr_in address, serverAddress;
    int sock, port;
    bool serverNotConnected;
    bool missionNotStarted;
    char ip[19];
    double latitude, longitude;
    time_t t, t1;
    struct tm tm;
    const char *courseName[6];
    const char *missionName[8];
    int courseIdx;
    int misiIdx;
    char teamID[6] = "ITSN";
    int sysMode;    /* 1 : Remote Operated
                     * 2 : Autonomous
                     * 3 : Killed */
    int calcCheckSum(char*);
    bool isResponseOK(char*);
    void sendDockMsg();
    void sendFlagMsg();
    int flag=0, dock=0;
    QMutex mtxx;

private slots:
    void heartBeat();
    void setDockNumber(int);
    void setFlagNumber(int);

signals:
    void sendLabel(QString);
    void terminateThread();
    void stopTimer();
};

#endif // TCPCONNECTION_H
