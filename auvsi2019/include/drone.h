#ifndef DRONE_H
#define DRONE_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QMutex>
#include <QString>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <common/mavlink.h>
#include <cmath>
#include <vector>

using namespace std;

/*
bit 0 : ignore position x
bit 1 : ignore position y
bit 2 : ignore position z
bit 3 : ignore velocity x
bit 4 : ignore velocity y
bit 5 : ignore velocity z
bit 6 : ignore acceleration x
bit 7 : ignore acceleration y
bit 8 : ignore acceleration z
bit 9 : ignore use force instead of acceleration
bit 10: ignore yaw
bit 11: ignore yaw rate

traveling state:
0: not traveling
1: goto latitude longitude
2: return to home/launch
3: takeoff
4: land
*/

typedef struct pos_t{
    double lat;
    double lon;
    double alt;
} Pos_t;

class Drone : public QObject
{
	Q_OBJECT
public:
    Drone(QObject *parent = Q_NULLPTR);
    ~Drone();
    void toggle_offboard_control(bool);
    void set_auto(bool);
    bool isOpened();
    void readMessage();
    void arm_disarm(bool);
    void takeoff(float, float=NAN); 
    void start_traveling();
    void goto_pos(float, float, float, float=NAN, float=0.5, float=0.5, float=0.5);
    void change_speed(int, float, int=-1);
    void return_to_home();
    void setWaypoint(unsigned, double, double);
    void deleteWaypoint(unsigned);
    int getTravelingState();

public slots:
    void initPixhawkConnection(std::vector<std::string>*);
    void getHomeLatitude(const QString&);
    void getHomeLongitude(const QString&);
    void disconnect();
    void mission_start(int, int);
    void setTraveling(int);
    void setRTL();
    void land();
    void launch();
    void checkArm();

signals:
    void sendPos(double, double);
    
private:
    int init_serial_port(const char*);
    void sendMessage(const mavlink_message_t&);
    void setInitPos();
    bool isArrived(int);
    void returnToLaunch();
    void initCondition();
    uint8_t cp;
    mavlink_heartbeat_t heartBeat;
    mavlink_status_t status;
    mavlink_message_t message, message_;
    mavlink_command_ack_t ack;
    mavlink_attitude_t attitude;
    mavlink_local_position_ned_t nedPos;
    mavlink_set_position_target_global_int_t home;
    mavlink_global_position_int_t global_pos;
    mavlink_set_position_target_local_ned_t setPos, initPos;
    bool opened;
    int result, fd, portNum;
    int system_id, companion_id, autopilot_id;
    const char *baudRate;
    time_t startTime;
    double latitude, longitude, compass, tmpLat, tmpLon, tmpCompass;
    double homeLatitude, homeLongitude;
    double launchLatitude, launchLongitude;
    vector< Pos_t > pos;
    int nextPos_idx;
    int travelingstate, rtl;
    uint32_t timestamp;
    const double MinRadius_lat = 0.000013;
    const double MinRadius_lon = 0.000013;
    double jarak;
    bool gotNedPos, gotGlobalPos;
    QMutex mtx;
};
#endif // !Drone_H