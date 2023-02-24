#ifndef GPSCONNECTION_H
#define GPSCONNECTION_H

#include <ros/ros.h>

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QTime>
#include <QTimer>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <common/mavlink.h>
#include <unordered_map>

#define PI 3.14159265

class GPSConnection : public QObject
{
	Q_OBJECT

public:
	GPSConnection(QObject *parent = Q_NULLPTR);
	~GPSConnection();
	bool isOpened();
	int getPortNum();
	float compassOlah;

	double lattDouble = 0.00000000000;
	double longDouble = 0.00000000000;

	char ambil_GPS[40];
	char lat_kirim[30];
	char long_kirim[30];
	char comp_kirim[10];

	float stof(const char* s);

signals:
	void serial_kirim_data_GPS(double, double, float);
	void sendCurrentPos(double, double, double);
	void sendCurrentPos(double, double);
	void sendAttitude(double);

protected:

private:
        ros::Publisher currentPosPub;
        ros::Subscriber nextWaypointSub;
        int n, result, fd;
		uint8_t cp;
		mavlink_status_t status;
		mavlink_message_t message;
		mavlink_global_position_int_t global_pos;
		mavlink_gps_raw_int_t raw_global_pos;
		mavlink_attitude_t attitude;
		mavlink_heartbeat_t heartBeat;
        unsigned char data[20];
		bool isKilled;
		int init_serial_port(const char *);
		int init_bluetooth(const char *);
		float pixhawkTemp;
		time_t startTime;
		bool opened;
		int portNum;
		double radToDegrees;
		const char *baudRate;
		double xGaussData,yGaussData,D,latitude,longitude;
		char inBuffer[100];
		const char *delim=",";

        // void readGPS();


public slots :
    	void run(std::vector<std::string>*);
    	void runFromPhone();
		void stop();
       //void sendDatatoSTM();
       //void retry_GPSConnection();
};

#endif // !GPSConnection_H
