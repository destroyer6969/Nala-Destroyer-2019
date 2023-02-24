#pragma once

#include <QTimer>
#include <QtWidgets/QMainWindow>
#include <qthread.h>
#include <qobject.h>
#include <qdebug.h>
#include <qpainter.h>
#include <math.h>
#include <qtextstream.h>
#include <qfile.h>
#include <qfiledialog.h>
#include <qtablewidget.h>
#include <qlcdnumber.h>
#include <qscrollarea.h>
#include <qpixmap.h>
#include <QPaintEvent>
#include <qjsonarray.h>
#include <qjsonobject.h>
#include <qjsondocument.h>
#include <qjsonvalue.h>
#include <QToolTip>
#include <QMessageBox>
#include <string>
#include <stdlib.h>
#include <object_detection/ObjectAttr.h>
#include "ui_MainWindow.h"
#include "missionwrapper.h"
#include "imgreceiver.h"
#include "gpsconnection.h"
#include "lidar.h"
#include "tcpconnection.h"
#include "srf.h"

#define NUM_MISI 11
#define PI 3.14159265

using namespace std;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();



private:
        
        Ui::MainWindow *ui;
        MissionWrapper *missionWrapper;
        ImgReceiver *imgReceiver;
        GPSConnection *gpsConnection;
        Lidar *lidar;
        TCPConnection *tcp;
        SRF *srf;
        QThread  *camThread, *missionWrapperThread, *gpsThread, *lidarThread, *tcpThread, *srfThread;
        PositionType tempPos;
        int *srfData;
        QPixmap *attitudeShip;
        QTimer *timer0;
        std::vector<string>portNames;
        QPen b,r,k;
        double boxLat, boxLon;

        // arrays of pointer to some widgets
        QSpinBox* comboBoxWaypoint[NUM_MISI];
        QTableWidget* tableWidgetWaypoint[NUM_MISI];
        ThresholdWidget* thresbox[NUM_COLORS];

        void initArrays();

        QPoint controlPoint;
        char buffer[70];
        char coreTemp[6][9];
        char gpuTemp[8];
        char tempPosString[13];
        char srfStringL[4], srfStringR[4], srfStringML[4], srfStringMR[4], srfStringS[4];

        // position coordinate
        double latitude_awal_[6], latitude_akhir_[6];
        double longitude_awal_[6], longitude_akhir_[6];
        double skala_latitude, skala_longitude;
        double latitude_awal, latitude_akhir;
        double longitude_awal, longitude_akhir;
        double rollAngle;
        int topLeftX, topLeftY, botLeftX, botLeftY;
        int topRightX, topRightY, botRightX, botRightY;
        int yThreshold;
        int controlLineIdx;
        void mirrorControlLine(int=0);

        void initPositions();
        void initConnection();
        void anotherInit();
        PositionType calculateCoordinate(const QPoint&);
        QPoint calculatePoint(const PositionType&);
        QPoint calculatePoint(double lat, double lon);

        // savers loaders
        QString filenameWaypoint;
        QString filename;

        // stop functions
        void stopCamThread();
        void stopMissionWrapperThread();
        void stopGPSThread();
        void stopSRFThread();
        void stopTCPThread();
        void setLocation(int, const char*);
        
private slots:
        void receiveFlagColor(int);
        void imageReceive(QImage*);
        void showInfoMap(QPoint&);
        void setWaypoint(QPoint&);
        void setWaypointFromMission(double, double, int);
        void deleteWaypointFromMission(int);
        void inputWaypoint();
        void setWaypointAndNext(QPoint&);
        void resetWaypoints();
        void resetWaypoints(unsigned);
        void resetAllWaypoints();
        void deleteWaypoint();
        void addWaypoint(unsigned);
        void setCurrPosAsWaypoint();
        void setPositionField(QPoint&);
        void loadWaypoint(unsigned, unsigned, PositionType&);
        void changeMissionSequence();
        void clearOutputs();
        void displayCurrentMission(int);
        void displayCurrentState(QString);
        void switchMissionTab(int);
        void openSensor();
        void closeSensor();
        void drawAttitude();
        void getAttitude(double);
        void sensor_input_output_inGUI_handler();
        void startMission();
        void endMission();
        void runTest();
        void stopTest();
        void setControlLine(bool, int, int, int, int);
        void setYThreshold(int);
        void connectDrone();
        void disconnectDrone();
        void receiveDronePos(double, double);
        void droneRTL();
        void droneLand();

//        tester
        void forward();
        void backward();
        void turnRight();
        void turnLeft();

        void onChangeNextDest(int, const PositionType*);
        void onUpdateSudutJarak(double, double, double);
        void changeCurrentPos(double, double, double);
        void setDanau8();
        void setAlpha();
        void setBravo();
        void setCharlie();
        void setDelta();
        void setRobotika();

//        savers and loaders
        void onSaveWaypoints();
        void onLoadWaypoints();
        void onSaveAsWaypoints();
        void onSave();
        void onLoad();
        void missionsAttributeLoaded(QJsonObject&);

//        buttons transition
        void onStartCam();
        void onStopCam();

//        thresholings
        void useThreshold(int);
        void binaryImageReceive(QImage* bin);

//        TCP Connection
        void openTCPConnection();
        void closeTCPConnection();

//        Camera Settings
        void setExposure(int);
        void setGain(int);
        void setWhiteBalance(int);

signals:
        void killImgReceiver();
        void waypointChanged(unsigned, unsigned, MapPoint);
        void waypointChanged(unsigned, unsigned, const PositionType&);
        void waypointLoaded(unsigned, unsigned, PositionType&);
        void currentPosChanged(const QPoint&);
        void sendSudutJarak(double, double, double);
        void startPhoneGPS();
	void startGPS(std::vector<std::string>*);
        void startLidar(std::vector<std::string>*);
        void startSRF(std::vector<std::string>*);
        void stopGPS();
        void stopLidar();
        void stopSRF();
        void newAngle(double);
        void sendLocation(double, double, double, double);
        void sendGMapPath(const char*);
        void stopTCP();
        void startTCP(const char*, int);
        void sendCourseIdx(int);
        void connectingDrone(std::vector<std::string>*);
        void disconnectingDrone();
        void dronePosChanged(const QPoint&);
        void droneSetTraveling(int);
        void droneCmdRTL();
        void droneCmdLand();
};