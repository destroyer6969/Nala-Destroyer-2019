#include "mainwindow.h"
#include <QByteArray>

using namespace std;


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);
        topLeftX = -5; topLeftY = -5;
        botLeftX = -5; botLeftY = -5;
        yThreshold = -5;
        b = QPen(Qt::blue, 3);
        r = QPen(Qt::red, 5);
        k = QPen(Qt::black, 2);
        mirrorControlLine(0);
        mirrorControlLine(1);
        setWindowTitle("AUVSI 2019");
        setWindowIcon(QIcon(":/images/Barunastra.png"));
        ui->labelBarunastra1->setPixmap(QPixmap(":/images/Barunastra.png"));
        
        missionWrapper = new MissionWrapper();
        missionWrapperThread = new QThread();
        missionWrapper->moveToThread(missionWrapperThread);
        missionWrapperThread->start();

        camThread = new QThread();
        imgReceiver = new ImgReceiver();
        imgReceiver->moveToThread(camThread);
        camThread->start();

        gpsThread = new QThread();
        gpsConnection = new GPSConnection();
        gpsConnection->moveToThread(gpsThread);
        gpsThread->start();

        lidar = new Lidar();
        lidarThread = new QThread();
        lidar->moveToThread(lidarThread);
        lidarThread->start();

        srf = new SRF();
        srfThread = new QThread();
        srf->moveToThread(srfThread);
        srfThread->start();

        tcp = new TCPConnection();
        tcpThread = new QThread();
        tcp->moveToThread(tcpThread);
        tcpThread->start();

        timer0 = new QTimer(0);

        initPositions();

        initArrays();
        initConnection();
        anotherInit();

        attitudeShip = new QPixmap(":images/roll_ship.png");
        qDebug() << "Main window created";
}

MainWindow::~MainWindow()
{
    timer0->stop();
    delete timer0;

    delete lidar;
    lidarThread->terminate();
//    delete lidarThread;
    
    stopCamThread();
    delete imgReceiver;
    camThread->wait(100);
    delete camThread;

    stopGPSThread();
    delete gpsConnection;
    gpsThread->wait(100);
    delete gpsThread;

    stopSRFThread();
    delete srf;
    srfThread->wait(100);
    delete srfThread;

    stopMissionWrapperThread();
    delete missionWrapper;
    missionWrapperThread->wait(100);
    delete missionWrapperThread;
    
    stopTCPThread();
    delete tcp;
    tcpThread->wait(100);
    delete tcpThread;

    qDebug() << "Main window destroyed";
}

void MainWindow::initPositions(){
    // init coordinate
    // danau 8
    latitude_awal_[0] = 29.213842;
    latitude_akhir_[0] = 29.213124;
    longitude_awal_[0]  = -81.019650;
    longitude_akhir_[0] = -81.018720;
    // alpha
    latitude_awal_[1] = 29.151829;
    latitude_akhir_[1] = 29.151129;
    longitude_awal_[1]  = -81.017489;
    longitude_akhir_[1] = -81.016618;
    //bravo
    latitude_awal_[2] = 29.152185;
    latitude_akhir_[2] = 29.151461;
    longitude_awal_[2]  = -81.017037;
    longitude_akhir_[2] = -81.016116;
    //charlie
    latitude_awal_[3] = 29.151890;
    latitude_akhir_[3] = 29.151108;
    longitude_awal_[3]  = -81.016776;
    longitude_akhir_[3] = -81.015789;
    // delta
    latitude_awal_[4] = 29.151633;
    latitude_akhir_[4] = 29.150786;
    longitude_awal_[4]  = -81.017540;
    longitude_akhir_[4] = -81.016463;
    // robotika
    latitude_awal_[5] = 29.222114;
    latitude_akhir_[5] = 29.221258;
    longitude_awal_[5]  = -81.027053;
    longitude_akhir_[5] = -81.025937;
}

void MainWindow::initArrays(){
    // init array comboBoxWaypoint
    comboBoxWaypoint[0]     = NULL;
    comboBoxWaypoint[1]     = ui->wpOptionsMisi_1;
    comboBoxWaypoint[2]     = ui->wpOptionsMisi_2;
    comboBoxWaypoint[3]     = ui->wpOptionsMisi_3;
    comboBoxWaypoint[4]     = ui->wpOptionsMisi_4;
    comboBoxWaypoint[5]     = ui->wpOptionsMisi_5;
    comboBoxWaypoint[6]     = ui->wpOptionsMisi_6;
    comboBoxWaypoint[7]     = ui->wpOptionsMisi_7;
    comboBoxWaypoint[8]     = ui->wpOptionsMisi_11;
    comboBoxWaypoint[9]     = ui->wpOptionsMisi_12;
    comboBoxWaypoint[10]    = ui->wpOptionsMisi_13;

    // init array tablewidgetwaypoint
    tableWidgetWaypoint[0]  = NULL;
    tableWidgetWaypoint[1]  = ui->tableWaypointMisi_1;
    tableWidgetWaypoint[2]  = ui->tableWaypointMisi_2;
    tableWidgetWaypoint[3]  = ui->tableWaypointMisi_3;
    tableWidgetWaypoint[4]  = ui->tableWaypointMisi_4;
    tableWidgetWaypoint[5]  = ui->tableWaypointMisi_5;
    tableWidgetWaypoint[6]  = ui->tableWaypointMisi_6;
    tableWidgetWaypoint[7]  = ui->tableWaypointMisi_7;
    tableWidgetWaypoint[8]  = ui->tableWaypointMisi_11;
    tableWidgetWaypoint[9]  = ui->tableWaypointMisi_12;
    tableWidgetWaypoint[10] = ui->tableWaypointMisi_13;

    // init array thresholding groupbox
    thresbox[0] = ui->tw0;
    thresbox[1] = ui->tw1;
    thresbox[2] = ui->tw3;
    thresbox[3] = ui->tw3;

    for(int i=0;i<6;i++)coreTemp[i][0]=0;
}

void MainWindow::initConnection(){
    connect(timer0, SIGNAL(timeout()), this, SLOT(sensor_input_output_inGUI_handler()), Qt::QueuedConnection);

//    Camera Settings
    connect(ui->spinBoxExposure,        SIGNAL(valueChanged(int)), this, SLOT(setExposure(int)), Qt::DirectConnection);
    connect(ui->spinBoxGain,            SIGNAL(valueChanged(int)), this, SLOT(setGain(int)), Qt::DirectConnection);
    connect(ui->spinBoxWhiteBalance ,   SIGNAL(valueChanged(int)), this, SLOT(setWhiteBalance(int)), Qt::DirectConnection);

//    1.1 MISI_CIRCUMNAV connection
    connect(ui->spinBoxSpeedMisi_11,    SIGNAL(valueChanged(int)), missionWrapper->misi[MISI_CIRCUMNAV], SLOT(setSpeed(int)));

//    1.2 MISI_MHEADING connection
    connect(ui->spinBoxSpeedMisi_12,    SIGNAL(valueChanged(int)), missionWrapper->misi[MISI_MHEADING], SLOT(setSpeed(int)));
    connect(ui->spinBoxSudutMisi_12,    SIGNAL(valueChanged(int)), missionWrapper->misi[MISI_MHEADING], SLOT(setSudut(int)));

//    1.3 MISI_SLALOM connection
    connect(ui->spinBoxSpeedMisi_13,    SIGNAL(valueChanged(int)), missionWrapper->misi[MISI_SLALOM], SLOT(setSpeed(int)));

//    1 MISI_AUTONAV connection
    connect(ui->spinBoxSpeedMisi_1,     SIGNAL(valueChanged(int)), missionWrapper->misi[MISI_AUTONAV], SLOT(setSpeed(int)), Qt::DirectConnection);
    connect(ui->spinBoxSudutMisi_1,     SIGNAL(valueChanged(int)), missionWrapper->misi[MISI_AUTONAV], SLOT(setSudut(int)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP1Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setP1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setI1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setD1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setP2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setI2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setD2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP3Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setP3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI3Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setI3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD3Misi_1,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setD3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setPD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setID(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setDD(double)), Qt::DirectConnection);
    connect(ui->delWp_1,                SIGNAL(clicked(bool)),        this,                               SLOT(deleteWaypoint()));
    connect(ui->addWp_1,                SIGNAL(clicked(bool)),        this,                               SLOT(setCurrPosAsWaypoint()));
    connect(ui->checkBoxMisi_1,         SIGNAL(stateChanged(int)),    missionWrapper->misi[MISI_AUTONAV], SLOT(setStatus(int)));
    connect(ui->breakTimeMisi_1,        SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTONAV], SLOT(setTimeout(double)));
    connect(ui->breakWaypointIndexMisi_1,SIGNAL(valueChanged(int)),   missionWrapper->misi[MISI_AUTONAV], SLOT(setBreakWPIndex(int)));
    connect(ui->breakUseCameraMisi_1,   SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_AUTONAV], SLOT(setUseCamera(int)));
    connect(missionWrapper->misi[MISI_AUTONAV], SIGNAL(setNewWaypoint(double, double, int)), this,             SLOT(setWaypointFromMission(double, double, int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_AUTONAV], SIGNAL(signalDeleteWaypoint(int)), this,                       SLOT(deleteWaypointFromMission(int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_AUTONAV], SIGNAL(displayCurrentState(QString)), this,                    SLOT(displayCurrentState(QString)), Qt::QueuedConnection);

//    2 MISI_SPEEDCHAL connection
    connect(ui->spinBoxSpeedMisi_2,     SIGNAL(valueChanged(int)),    missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setSpeed(int)));
    connect(ui->doubleSpinBoxP1Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setP1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setI1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setD1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setP2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setI2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setD2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP3Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setP3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI3Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setI3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD3Misi_2,  SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setD3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance_2, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setP4(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance_2, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setI4(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance_2, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setD4(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setPD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setID(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance, SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setDD(double)), Qt::DirectConnection);
    connect(ui->spinBoxYThresholdMisi_2,  SIGNAL(valueChanged(int)),    missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setYThreshold(int)), Qt::DirectConnection);
    connect(ui->spinBoxServoMuter,        SIGNAL(valueChanged(int)),    missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setServoMuterMax(int)), Qt::DirectConnection);
    connect(ui->delWp_2,               SIGNAL(clicked(bool)),        this,                                 SLOT(deleteWaypoint()));
    connect(ui->addWp_2,               SIGNAL(clicked(bool)),        this,                                 SLOT(setCurrPosAsWaypoint()));
    connect(ui->checkBoxMisi_2,        SIGNAL(stateChanged(int)),    missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setStatus(int)));
    connect(ui->breakTimeMisi_2,        SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setTimeout(double)));
    connect(ui->breakWaypointIndexMisi_2,SIGNAL(valueChanged(int)),   missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setBreakWPIndex(int)));
    connect(ui->breakUseCameraMisi_2,   SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_SPEEDCHAL], SLOT(setUseCamera(int)));
    connect(missionWrapper->misi[MISI_SPEEDCHAL], SIGNAL(setNewWaypoint(double, double, int)), this,             SLOT(setWaypointFromMission(double, double, int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_SPEEDCHAL], SIGNAL(signalDeleteWaypoint(int)), this,                       SLOT(deleteWaypointFromMission(int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_SPEEDCHAL], SIGNAL(displayCurrentState(QString)), this,                    SLOT(displayCurrentState(QString)), Qt::QueuedConnection);

//    3 MISI_AUTODOCK connection
    connect(ui->spinBoxSpeedMisi_3,         SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setSpeed(int)), Qt::DirectConnection);
    connect(ui->spinBoxSpeedSlowMisi_3,     SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setSpeedSlow(int)), Qt::DirectConnection);
    connect(ui->spinBoxThresholdPinger,     SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setThreshold(int)), Qt::DirectConnection);
    connect(ui->spinBoxThresholdFrequency,  SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setFrequency(int)), Qt::DirectConnection);
    connect(ui->spinBoxSudutMisi_3,         SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setSudut(int)), Qt::DirectConnection);
    connect(ui->spinBoxMaxPinger,           SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setMaxPinger(int)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP1Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setP1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setI1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setD1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setP2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setI2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setD2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP3Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setP3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI3Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setI3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD3Misi_3,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setD3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setPD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setID(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setDD(double)), Qt::DirectConnection);
    connect(ui->spinBoxLidarControlRangeMisi_3, SIGNAL(valueChanged(int)),  missionWrapper->misi[MISI_AUTODOCK], SLOT(setLidarControlRange(int)), Qt::DirectConnection);
    connect(ui->delWp_3,                    SIGNAL(clicked(bool)),          this,                                SLOT(deleteWaypoint()), Qt::DirectConnection);
    connect(ui->addWp_3,                    SIGNAL(clicked(bool)),          this,                                SLOT(setCurrPosAsWaypoint()));
    connect(ui->checkBoxMisi_3,             SIGNAL(stateChanged(int)),      missionWrapper->misi[MISI_AUTODOCK], SLOT(setStatus(int)));
    connect(ui->breakTimeMisi_3,        SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_AUTODOCK], SLOT(setTimeout(double)));
    connect(ui->breakWaypointIndexMisi_3,SIGNAL(valueChanged(int)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setBreakWPIndex(int)));
    connect(ui->breakUseCameraMisi_3,   SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setUseCamera(int)));
    connect(ui->checkBoxUseHydrophone,  SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_AUTODOCK], SLOT(setUseSomething(int)));
    connect(missionWrapper->misi[MISI_AUTODOCK], SIGNAL(setNewWaypoint(double, double, int)), this,             SLOT(setWaypointFromMission(double, double, int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_AUTODOCK], SIGNAL(signalDeleteWaypoint(int)), this,                       SLOT(deleteWaypointFromMission(int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_AUTODOCK], SIGNAL(displayCurrentState(QString)), this,                    SLOT(displayCurrentState(QString)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_AUTODOCK], SIGNAL(sendDock(int)), tcp, SLOT(setDockNumber(int)), Qt::DirectConnection);

//    4 MISI_RAISETHEFLAG connection
    connect(ui->spinBoxSpeedMisi_4,         SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setSpeed(int)), Qt::DirectConnection);
    connect(ui->spinBoxSpeedSlowMisi_4,     SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setSpeedSlow(int)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP1Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setP1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setI1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setD1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setP2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setI2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setD2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP3Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setP3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI3Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setI3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD3Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setD3(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP4Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setP4(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI4Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setI4(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD4Misi_4,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setD4(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setPD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setID(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setDD(double)), Qt::DirectConnection);
    connect(ui->spinBoxLidarControlRangeMisi_4, SIGNAL(valueChanged(int)),  missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setLidarControlRange(int)), Qt::DirectConnection);
    connect(ui->delWp_4,                    SIGNAL(clicked(bool)),          this,                                    SLOT(deleteWaypoint()));
    connect(ui->addWp_4,                    SIGNAL(clicked(bool)),          this,                                    SLOT(setCurrPosAsWaypoint()));
    connect(ui->checkBoxMisi_4,             SIGNAL(stateChanged(int)),      missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setStatus(int)));
    connect(ui->breakTimeMisi_4,        SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setTimeout(double)));
    connect(ui->breakWaypointIndexMisi_4,SIGNAL(valueChanged(int)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setBreakWPIndex(int)));
    connect(ui->breakUseCameraMisi_4,   SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setUseCamera(int)));
    connect(ui->pushButtonConnectDrone, SIGNAL(clicked(bool)),       this, SLOT(connectDrone()));
    connect(ui->pushButtonDisconnectDrone, SIGNAL(clicked(bool)),    this, SLOT(disconnectDrone()));
    connect(ui->droneHomeLatitude, SIGNAL(textChanged(const QString &)), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(getHomeLatitude(const QString&)));
    connect(ui->droneHomeLongitude, SIGNAL(textChanged(const QString &)), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(getHomeLongitude(const QString&)));
    connect(ui->checkBoxUseDrone,  SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setUseSomething(int)), Qt::DirectConnection);
    connect(ui->checkBoxUseLidar,  SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(setUseSomething2(int)), Qt::DirectConnection);
    connect(missionWrapper->misi[MISI_RAISETHEFLAG], SIGNAL(setNewWaypoint(double, double, int)), this,             SLOT(setWaypointFromMission(double, double, int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_RAISETHEFLAG], SIGNAL(signalDeleteWaypoint(int)), this,                       SLOT(deleteWaypointFromMission(int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_RAISETHEFLAG], SIGNAL(displayCurrentState(QString)), this,                    SLOT(displayCurrentState(QString)), Qt::QueuedConnection);
    connect(this, SIGNAL(connectingDrone(std::vector<std::string>*)), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(initPixhawkConnection(std::vector<std::string>*)), Qt::QueuedConnection);
    connect(this, SIGNAL(disconnectingDrone()), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(disconnect()), Qt::DirectConnection);
    connect(missionWrapper->misi[MISI_RAISETHEFLAG], SIGNAL(sendFlag(int)), tcp, SLOT(setFlagNumber(int)), Qt::DirectConnection);

//    5 MISI_FINDPATH connection
    connect(ui->spinBoxSpeedMisi_5,           SIGNAL(valueChanged(int)),          missionWrapper->misi[MISI_FINDPATH], SLOT(setSpeed(int)),     Qt::DirectConnection);
    connect(ui->spinBoxSpeedSlowMisi_5,       SIGNAL(valueChanged(int)),          missionWrapper->misi[MISI_FINDPATH], SLOT(setSpeedSlow(int)), Qt::DirectConnection);
    connect(ui->comboBoxModeMisi_5,           SIGNAL(currentIndexChanged(int)),   missionWrapper->misi[MISI_FINDPATH], SLOT(setMode(int)),      Qt::DirectConnection);
    connect(ui->doubleSpinBoxP1Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setP1(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setI1(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setD1(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setP2(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setI2(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setD2(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxP3Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setP3(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxI3Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setI3(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxD3Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setD3(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxP4Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setP4(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxI4Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setI4(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxD4Misi_5,        SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setD4(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance,       SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setPD(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance,       SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setID(double)),     Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance,       SIGNAL(valueChanged(double)),       missionWrapper->misi[MISI_FINDPATH], SLOT(setDD(double)),     Qt::DirectConnection);
    connect(ui->spinBoxYThresholdMisi_5,    SIGNAL(valueChanged(int)),          missionWrapper->misi[MISI_FINDPATH], SLOT(setYThreshold(int)), Qt::DirectConnection);
    connect(ui->spinBoxServoMuter_2,        SIGNAL(valueChanged(int)),          missionWrapper->misi[MISI_FINDPATH], SLOT(setServoMuterMax(int)), Qt::DirectConnection);
    connect(ui->delWp_5,                    SIGNAL(clicked(bool)),              this,                                SLOT(deleteWaypoint()));
    connect(ui->addWp_5,                    SIGNAL(clicked(bool)),              this,                                SLOT(setCurrPosAsWaypoint()));
    connect(ui->checkBoxMisi_5,             SIGNAL(stateChanged(int)),          missionWrapper->misi[MISI_FINDPATH], SLOT(setStatus(int)));
    connect(ui->breakTimeMisi_5,        SIGNAL(valueChanged(double)), missionWrapper->misi[MISI_FINDPATH], SLOT(setTimeout(double)));
    connect(ui->breakWaypointIndexMisi_5,SIGNAL(valueChanged(int)),   missionWrapper->misi[MISI_FINDPATH], SLOT(setBreakWPIndex(int)));
    connect(ui->breakUseCameraMisi_5,   SIGNAL(stateChanged(int)),   missionWrapper->misi[MISI_FINDPATH], SLOT(setUseCamera(int)));
    connect(missionWrapper->misi[MISI_FINDPATH], SIGNAL(setNewWaypoint(double, double, int)), this,             SLOT(setWaypointFromMission(double, double, int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_FINDPATH], SIGNAL(signalDeleteWaypoint(int)), this,                       SLOT(deleteWaypointFromMission(int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_FINDPATH], SIGNAL(displayCurrentState(QString)), this,                    SLOT(displayCurrentState(QString)), Qt::QueuedConnection);

//    6 MISI_FOLLOWLEADER connection
    connect(ui->spinBoxSpeedMisi_6,         SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setSpeed(int)), Qt::DirectConnection);
    // connect(ui->spinBoxSpeedBawahMisi_6,    SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setWaypoint(int)));
    connect(ui->doubleSpinBoxP1Misi_6,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setP1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_6,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setI1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_6,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setD1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_6,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setP2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_6,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setI2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_6,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setD2(double)), Qt::DirectConnection);
    connect(ui->delWp_6,                    SIGNAL(clicked(bool)),          this,                                    SLOT(deleteWaypoint()));
    connect(ui->addWp_6,                    SIGNAL(clicked(bool)),          this,                                    SLOT(setCurrPosAsWaypoint()));
    connect(ui->checkBoxMisi_6,             SIGNAL(stateChanged(int)),      missionWrapper->misi[MISI_FOLLOWLEADER], SLOT(setStatus(int)));

    connect(missionWrapper->misi[MISI_FOLLOWLEADER],    SIGNAL(sendFlagColor(int)), this, SLOT(receiveFlagColor(int)));


//    7 MISI_RETURNHOME connection
    connect(ui->spinBoxSpeedMisi_7,         SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_RETURNHOME], SLOT(setSpeed(int)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP1Misi_7,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setP1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI1Misi_7,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setI1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD1Misi_7,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setD1(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxP2Misi_7,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setP2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxI2Misi_7,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setI2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxD2Misi_7,      SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setD2(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setPD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setID(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setDD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxPDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setPD(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxIDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setID(double)), Qt::DirectConnection);
    connect(ui->doubleSpinBoxDDistance,     SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setDD(double)), Qt::DirectConnection);
    connect(ui->delWp_7,                    SIGNAL(clicked(bool)),          this,                                  SLOT(deleteWaypoint()));
    connect(ui->addWp_7,                    SIGNAL(clicked(bool)),          this,                                  SLOT(setCurrPosAsWaypoint()));
    connect(ui->breakTimeMisi_6,            SIGNAL(valueChanged(double)),   missionWrapper->misi[MISI_RETURNHOME], SLOT(setTimeout(double)));
    connect(ui->breakWaypointIndexMisi_6,   SIGNAL(valueChanged(int)),      missionWrapper->misi[MISI_RETURNHOME], SLOT(setBreakWPIndex(int)));
    connect(ui->breakUseCameraMisi_6,       SIGNAL(stateChanged(int)),      missionWrapper->misi[MISI_RETURNHOME], SLOT(setUseCamera(int)));
    connect(missionWrapper->misi[MISI_RETURNHOME], SIGNAL(setNewWaypoint(double, double, int)), this,             SLOT(setWaypointFromMission(double, double, int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_RETURNHOME], SIGNAL(signalDeleteWaypoint(int)), this,                       SLOT(deleteWaypointFromMission(int)), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_RETURNHOME], SIGNAL(displayCurrentState(QString)), this,                    SLOT(displayCurrentState(QString)), Qt::QueuedConnection);

// drone's
    connect(ui->delWp_11, SIGNAL(clicked(bool)), this, SLOT(deleteWaypoint()));
    connect(ui->pushButtonLand, SIGNAL(clicked(bool)), this, SLOT(droneLand()));
    connect(ui->pushButtonLand, SIGNAL(clicked(bool)), missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(droneCamOff()), Qt::QueuedConnection);
    connect(ui->pushButtonRTL, SIGNAL(clicked(bool)), this, SLOT(droneRTL()));
    connect(ui->pushButtonArm, SIGNAL(clicked(bool)), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(checkArm()), Qt::QueuedConnection);
    connect(ui->pushButtonArm, SIGNAL(clicked(bool)), missionWrapper->misi[MISI_RAISETHEFLAG], SLOT(droneCamOn()), Qt::QueuedConnection);
    connect(this, SIGNAL(droneSetTraveling(int)), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(setTraveling(int)), Qt::DirectConnection);
    // connect(this, SIGNAL(droneCmdLand()), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(land()), Qt::QueuedConnection);
    // connect(this, SIGNAL(droneCmdRTL()), missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SLOT(setRTL()), Qt::QueuedConnection);
    connect(missionWrapper->misi[MISI_RAISETHEFLAG]->drone, SIGNAL(sendPos(double, double)), this, SLOT(receiveDronePos(double, double)), Qt::QueuedConnection);
    connect(this, SIGNAL(dronePosChanged(const QPoint&)), ui->labelMapping, SLOT(changeDronePos(const QPoint&)));
    connect(missionWrapper->misi[MISI_RAISETHEFLAG], SIGNAL(switchCam(int)), imgReceiver, SLOT(switchCam(int)), Qt::DirectConnection);
//    SENSORS
    connect(ui->pushButtonOpenSensor,   SIGNAL(clicked(bool)), this, SLOT(openSensor()));
    connect(ui->pushButtonCloseSensor,  SIGNAL(clicked(bool)), this, SLOT(closeSensor()));

//    IMAGE connection
    connect(ui->startCamButton, SIGNAL(clicked(bool)),          imgReceiver,    SLOT(start()));
    connect(ui->stopCamButton,  SIGNAL(clicked(bool)),          imgReceiver,    SLOT(stop()),               Qt::DirectConnection);
    connect(ui->startCamButton, SIGNAL(clicked(bool)),          this,           SLOT(onStartCam()));
    connect(ui->stopCamButton,  SIGNAL(clicked(bool)),          this,           SLOT(onStopCam()));
    connect(imgReceiver,        SIGNAL(imgReceived(QImage*)),   this,           SLOT(imageReceive(QImage*)),Qt::DirectConnection);
    connect(this,               SIGNAL(killImgReceiver()),      imgReceiver,    SLOT(kill()),               Qt::DirectConnection);
    connect(imgReceiver,        SIGNAL(sendDetectedObject(const object_detection::ObjectAttr&)),      missionWrapper,    SLOT(getDetectedObject(const object_detection::ObjectAttr&)),               Qt::DirectConnection);

//    Lidar connection
    connect(lidar,                      SIGNAL(sendImage(QPixmap*)), ui->labelMapping, SLOT(setMapImage(QPixmap*)), Qt::DirectConnection);
    connect(lidar,                      SIGNAL(requestImage()),      ui->labelMapping, SLOT(sendingImage()), Qt::DirectConnection);
    connect(ui->labelMapping,           SIGNAL(sendImage(QPixmap*)), lidar,            SLOT(getImageMap(QPixmap*)), Qt::DirectConnection);
    connect(this,                       SIGNAL(startLidar(std::vector<std::string>*)),        lidar,            SLOT(run(std::vector<std::string>*)), Qt::QueuedConnection);
    connect(this,                       SIGNAL(stopLidar()),         lidar,            SLOT(stop()), Qt::DirectConnection);
    connect(lidar,                      SIGNAL(obstacle(double, double)), missionWrapper, SLOT(getLidarObstacle(double, double)), Qt::DirectConnection);
    

//    GPS connection
    connect(ui->labelMapping,           SIGNAL(sendMousePosition(QPoint&)), this,               SLOT(showInfoMap(QPoint&)));
    connect(ui->labelMapping,           SIGNAL(leftClicked(QPoint&)),       this,               SLOT(setWaypoint(QPoint&)));
    connect(ui->labelMapping,           SIGNAL(rightClicked(QPoint&)),      this,               SLOT(setPositionField(QPoint&)));
    connect(ui->labelMapping,           SIGNAL(doubleClicked(QPoint&)),     this,               SLOT(setWaypointAndNext(QPoint&)));
    connect(ui->checkboxLabelMap,       SIGNAL(stateChanged(int)),          ui->labelMapping,   SLOT(toggleLabel(int)));
    connect(ui->checkboxMap,            SIGNAL(stateChanged(int)),          ui->labelMapping,   SLOT(mapMode(int)), Qt::DirectConnection);
    connect(ui->pushButtonInputWaypoint,SIGNAL(clicked(bool)),              this,               SLOT(inputWaypoint()));
    connect(ui->pushButtonReset,        SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(clearCircles()));
    connect(ui->pushButtonReset,        SIGNAL(clicked(bool)),              this,               SLOT(resetAllWaypoints()));
    connect(ui->pushButtonRotateRight,  SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(rotateMapRight()));
    connect(ui->pushButtonRotateLeft,   SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(rotateMapLeft()));
    connect(ui->pushButtonMoveUp,       SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(translateMapUp()));
    connect(ui->pushButtonMoveRight,    SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(translateMapRight()));
    connect(ui->pushButtonMoveDown,     SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(translateMapDown()));
    connect(ui->pushButtonMoveLeft,     SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(translateMapLeft()));
    connect(ui->pushButtonMoveCenter,   SIGNAL(clicked(bool)),              ui->labelMapping,   SLOT(resetTransformation()));
    connect(this,                       SIGNAL(startPhoneGPS()),            gpsConnection,      SLOT(runFromPhone()), Qt::QueuedConnection);
    connect(this,                       SIGNAL(startGPS(std::vector<std::string>*)),            gpsConnection,      SLOT(run(std::vector<std::string>*)), Qt::QueuedConnection);
    connect(this,                       SIGNAL(stopGPS()),                  gpsConnection,      SLOT(stop()), Qt::DirectConnection);

    connect(gpsConnection, SIGNAL(sendCurrentPos(double, double, double)),  missionWrapper, SLOT(getCurrentPos(double, double, double)), Qt::DirectConnection);
    connect(gpsConnection, SIGNAL(sendCurrentPos(double, double, double)),  this,           SLOT(changeCurrentPos(double, double, double)), Qt::DirectConnection);
    connect(gpsConnection, SIGNAL(sendCurrentPos(double, double, double)),  lidar,          SLOT(getCurrentPos(double, double, double)), Qt::DirectConnection);
    connect(gpsConnection, SIGNAL(sendCurrentPos(double, double)),          tcp,            SLOT(getCurrentPos(double, double)), Qt::DirectConnection);
    connect(gpsConnection, SIGNAL(sendAttitude(double)),                    this,           SLOT(getAttitude(double)), Qt::DirectConnection);
    connect(this,       SIGNAL(currentPosChanged(const QPoint&)),               ui->labelMapping,   SLOT(changeInitCircle(const QPoint&)), Qt::QueuedConnection);
    connect(this,       SIGNAL(newAngle(double)),                               ui->labelMapping,   SLOT(getNewAngle(double)), Qt::DirectConnection);

    connect(this, SIGNAL(waypointChanged(unsigned, unsigned, const PositionType&)), missionWrapper,            SLOT(setWaypoint(unsigned, unsigned, const PositionType&)), Qt::DirectConnection);
    connect(this, SIGNAL(waypointChanged(unsigned, unsigned, MapPoint)),            ui->labelMapping,   SLOT(addCircle(unsigned, unsigned, MapPoint)));
    connect(this, SIGNAL(waypointLoaded(unsigned, unsigned, PositionType&)),        this,               SLOT(loadWaypoint(unsigned, unsigned, PositionType&)));

    // related to missions
    connect(missionWrapper, SIGNAL(nextDestChanged(int, const PositionType*)), this, SLOT(onChangeNextDest(int, const PositionType*)), Qt::QueuedConnection);
    connect(missionWrapper, SIGNAL(sendSudutJarak(double, double, double)),  this, SLOT(onUpdateSudutJarak(double, double, double)), Qt::QueuedConnection);
    connect(missionWrapper, SIGNAL(endMission()), this, SLOT(endMission()), Qt::QueuedConnection);
    connect(ui->pushButtonStartMission, SIGNAL(clicked(bool)), this, SLOT(startMission()));
    connect(ui->pushButtonStartMission, SIGNAL(clicked(bool)), missionWrapper, SLOT(run()), Qt::QueuedConnection);
    connect(ui->pushButtonStopMission, SIGNAL(clicked(bool)), missionWrapper, SLOT(forceStop(bool)), Qt::DirectConnection);
    connect(ui->pushButtonNextMission, SIGNAL(clicked(bool)), missionWrapper, SLOT(nextMission()), Qt::DirectConnection);
    connect(ui->pushButtonStopMission, SIGNAL(clicked(bool)), ui->labelMapping, SLOT(unvisitAllWaypoints()));
    connect(ui->pushButtonPanic,       SIGNAL(clicked(bool)), missionWrapper, SLOT(panic()), Qt::DirectConnection);
    connect(missionWrapper, SIGNAL(nextMission(int)), ui->labelMapping, SLOT(setNextMisi(int)), Qt::QueuedConnection);
    connect(missionWrapper, SIGNAL(nextMission(int)), tcp,              SLOT(setMission(int)), Qt::QueuedConnection);
    connect(missionWrapper, SIGNAL(sendControlLine(bool, int, int, int, int)), this, SLOT(setControlLine(bool, int, int, int, int)), Qt::QueuedConnection);
    connect(missionWrapper, SIGNAL(sendYThreshold(int)), this, SLOT(setYThreshold(int)), Qt::QueuedConnection);
    connect(ui->pushButtonChangeMissionSequence, SIGNAL(clicked(bool)), this, SLOT(changeMissionSequence()));
    connect(missionWrapper, SIGNAL(clearOutputs()), this, SLOT(clearOutputs()));
    connect(missionWrapper, SIGNAL(displayCurrentMission(int)), this, SLOT(displayCurrentMission(int)));
    connect(missionWrapper, SIGNAL(switchMissionTab(int)), this, SLOT(switchMissionTab(int)));
    connect(missionWrapper, SIGNAL(sendSRFMode(std::string&)), srf, SLOT(setSRFMode(std::string&)), Qt::DirectConnection);
    connect(ui->spinBoxTengahServoMotor, SIGNAL(valueChanged(int)), missionWrapper, SLOT(setTengahServoMotor(int)), Qt::DirectConnection);
    connect(ui->spinBoxTengahServoKamera, SIGNAL(valueChanged(int)), missionWrapper, SLOT(setTengahServoKamera(int)), Qt::DirectConnection);
    connect(tcp->timerHeartBeat,          SIGNAL(timeout()),         missionWrapper,    SLOT(readFromSTM()), Qt::DirectConnection);

//    SAVERS LOADERS connection
    connect(ui->pushButtonSaveWaypoint,     SIGNAL(clicked(bool)), this, SLOT(onSaveWaypoints()));
    connect(ui->pushButtonSaveAsWaypoint,   SIGNAL(clicked(bool)), this, SLOT(onSaveAsWaypoints()));
    connect(ui->pushButtonLoadWaypoint,     SIGNAL(clicked(bool)), this, SLOT(onLoadWaypoints()));
    connect(ui->actionSave,                 SIGNAL(triggered()),   this, SLOT(onSave()));
    connect(ui->actionOpen,                 SIGNAL(triggered()),   this, SLOT(onLoad()));

//     Location connection
    connect(ui->pushButtonLocationDanau8,   SIGNAL(clicked(bool)),  this,   SLOT(setDanau8()));
    connect(ui->pushButtonLocationAlpha,    SIGNAL(clicked(bool)),  this,   SLOT(setAlpha()));
    connect(ui->pushButtonLocationBravo,    SIGNAL(clicked(bool)),  this,   SLOT(setBravo()));
    connect(ui->pushButtonLocationCharlie,  SIGNAL(clicked(bool)),  this,   SLOT(setCharlie()));
    connect(ui->pushButtonLocationDelta,    SIGNAL(clicked(bool)),  this,   SLOT(setDelta()));
    connect(ui->pushButtonLocationRobotika, SIGNAL(clicked(bool)),  this,   SLOT(setRobotika()));
    connect(this,                           SIGNAL(sendLocation(double, double, double, double)),  lidar,   SLOT(getLocation(double, double, double, double)), Qt::DirectConnection); 
    connect(this,                           SIGNAL(sendGMapPath(const char*)), ui->labelMapping, SLOT(getGMap(const char*)), Qt::DirectConnection);
    connect(this,                           SIGNAL(sendCourseIdx(int)), tcp, SLOT(setCourse(int)), Qt::DirectConnection);

//    Thresholding connection
    connect(ui->groupBoxThres, SIGNAL(currentChanged(int)), imgReceiver, SLOT(colorChange(int)), Qt::DirectConnection);
    connect(ui->checkBoxThreshold, SIGNAL(stateChanged(int)), this, SLOT(useThreshold(int)));
    connect(imgReceiver, SIGNAL(sendBinaryImage(QImage*)), this, SLOT(binaryImageReceive(QImage*)), Qt::DirectConnection);
    for(int i=0; i<NUM_COLORS; i++){
        connect(thresbox[i]->hminSlider,    SIGNAL(valueChanged(int)), imgReceiver, SLOT(hminChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->sminSlider,    SIGNAL(valueChanged(int)), imgReceiver, SLOT(sminChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->vminSlider,    SIGNAL(valueChanged(int)), imgReceiver, SLOT(vminChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->hmaxSlider,    SIGNAL(valueChanged(int)), imgReceiver, SLOT(hmaxChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->smaxSlider,    SIGNAL(valueChanged(int)), imgReceiver, SLOT(smaxChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->vmaxSlider,    SIGNAL(valueChanged(int)), imgReceiver, SLOT(vmaxChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->erodeSlider,   SIGNAL(valueChanged(int)), imgReceiver, SLOT(erodeChange(int)), Qt::DirectConnection);
        connect(thresbox[i]->dilateSlider,  SIGNAL(valueChanged(int)), imgReceiver, SLOT(dilateChange(int)), Qt::DirectConnection);
    }

//     Tester Connection
    connect(ui->startTest,                SIGNAL(clicked(bool)),     this,             SLOT(runTest()));
    connect(ui->startTest,                SIGNAL(clicked(bool)),     missionWrapper,   SLOT(runTest()), Qt::QueuedConnection);
    connect(ui->stopTest,                 SIGNAL(clicked(bool)),     this,             SLOT(stopTest()));
    connect(ui->stopTest,                 SIGNAL(clicked(bool)),     missionWrapper,   SLOT(stopTest()), Qt::DirectConnection);
    connect(ui->spinBoxMotorKananTester,  SIGNAL(valueChanged(int)), missionWrapper,   SLOT(setMotorKananTest(int)), Qt::DirectConnection);
    connect(ui->spinBoxMotorKiriTester,   SIGNAL(valueChanged(int)), missionWrapper,   SLOT(setMotorKiriTest(int)), Qt::DirectConnection);
    connect(ui->spinBoxServoKananTester,  SIGNAL(valueChanged(int)), missionWrapper,   SLOT(setServoKananTest(int)), Qt::DirectConnection);
    connect(ui->spinBoxServoKiriTester,   SIGNAL(valueChanged(int)), missionWrapper,   SLOT(setServoKiriTest(int)), Qt::DirectConnection);
    connect(ui->spinBoxServoKameraTester, SIGNAL(valueChanged(int)), missionWrapper,   SLOT(setServoKameraTest(int)), Qt::DirectConnection);
    connect(ui->pushButtonForward,        SIGNAL(clicked(bool)),     this,             SLOT(forward()));
    connect(ui->pushButtonBackward,       SIGNAL(clicked(bool)),     this,             SLOT(backward()));
    connect(ui->pushButtonTurnRight,      SIGNAL(clicked(bool)),     this,             SLOT(turnRight()));
    connect(ui->pushButtonTurnLeft,       SIGNAL(clicked(bool)),     this,             SLOT(turnLeft()));

//     TCP Connection
    connect(ui->btnStartConnServer,       SIGNAL(clicked(bool)),                this,   SLOT(openTCPConnection()));
    connect(ui->btnStopConnServer,        SIGNAL(clicked(bool)),                this,   SLOT(closeTCPConnection()));
    connect(this,                         SIGNAL(startTCP(const char*, int)),   tcp,    SLOT(openConnection(const char*, int)),     Qt::QueuedConnection);
    connect(this,                         SIGNAL(stopTCP()),                    tcp,    SLOT(closeConnection()),     Qt::DirectConnection);
    connect(ui->pushButtonStartMission,   SIGNAL(clicked(bool)),                tcp,    SLOT(startConnection()), Qt::QueuedConnection);
    connect(tcp,                          SIGNAL(sendLabel(QString)),           ui->connectionState,      SLOT(setText(QString)));
    connect(missionWrapper,               SIGNAL(newSysMode(int)),              tcp,    SLOT(setSysMode(int)), Qt::DirectConnection);
    connect(tcp->timerHeartBeat,          SIGNAL(timeout()),                    tcp,    SLOT(heartBeat()));

//    SRF Connection
    connect(this, SIGNAL(startSRF(std::vector<std::string>*)), srf, SLOT(run(std::vector<std::string>*)), Qt::QueuedConnection);
    connect(this, SIGNAL(stopSRF()), srf, SLOT(stop()), Qt::DirectConnection);
    connect(srf, SIGNAL(sendData(int, int, int, int, int)), missionWrapper, SLOT(srfDataToMission(int, int, int, int, int)), Qt::DirectConnection);
    
//    Map Connection
    connect(ui->labelMapping,             SIGNAL(updateAttitude()),   this,       SLOT(drawAttitude()));
    connect(ui->resetMap,                 SIGNAL(clicked(bool)), ui->labelMapping, SLOT(resetMap()));
    connect(ui->checkBoxPathMap,          SIGNAL(stateChanged(int)), ui->labelMapping, SLOT(togglePath(int)));
}

void MainWindow::anotherInit(){
    ui->lineEditCurrentMission->setFont(QFont("Courier New", 10));
    ui->labelMapping->toggleLabel(ui->checkboxLabelMap->isChecked());
    ui->labelMapping->togglePath(ui->checkBoxPathMap->isChecked());
    setLocation(0, ":images/danau8.jpg");
    ui->labelMapping->mapMode(ui->checkboxMap->isChecked());
    lidar->initImageMap();
    tempPos.latitude = tempPos.longitude = NAN;
    ui->btnStopConnServer->setDisabled(true);
    ui->pushButtonCloseSensor->setDisabled(true);
    ui->stopTest->setDisabled(true);
    ui->pushButtonDisconnectDrone->setDisabled(true);
    ui->pushButtonPanic->setDisabled(true);
    ui->tabWidgetMisi->setCurrentIndex(1);
    controlLineIdx = 0;
    srfData = srf->getSRFData();
    missionWrapper->setMotorKananTest(ui->spinBoxMotorKananTester->value());
    missionWrapper->setMotorKiriTest(ui->spinBoxMotorKiriTester->value());
    missionWrapper->setServoKananTest(ui->spinBoxServoKananTester->value());
    missionWrapper->setServoKiriTest(ui->spinBoxServoKiriTester->value());
    missionWrapper->setServoKameraTest(ui->spinBoxServoKameraTester->value());
    missionWrapper->setTengahServoMotor(ui->spinBoxTengahServoMotor->value());
    missionWrapper->setTengahServoKamera(ui->spinBoxTengahServoKamera->value());
    changeMissionSequence();
    endMission();
    timer0->start(100);
    missionWrapper->misi[MISI_AUTONAV]->setStatus(ui->checkBoxMisi_1->isChecked());
    missionWrapper->misi[MISI_AUTONAV]->setP1(ui->doubleSpinBoxP1Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setI1(ui->doubleSpinBoxI1Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setD1(ui->doubleSpinBoxD1Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setP2(ui->doubleSpinBoxP2Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setI2(ui->doubleSpinBoxI2Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setD2(ui->doubleSpinBoxD2Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setP3(ui->doubleSpinBoxP3Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setI3(ui->doubleSpinBoxI3Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setD3(ui->doubleSpinBoxD3Misi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_AUTONAV]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_AUTONAV]->setDD(ui->doubleSpinBoxDDistance->value());
    missionWrapper->misi[MISI_AUTONAV]->setSpeed(ui->spinBoxSpeedMisi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setTimeout(ui->breakTimeMisi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setBreakWPIndex(ui->breakWaypointIndexMisi_1->value());
    missionWrapper->misi[MISI_AUTONAV]->setUseCamera(ui->breakUseCameraMisi_1->isChecked());

    missionWrapper->misi[MISI_SPEEDCHAL]->setStatus(ui->checkBoxMisi_2->isChecked());
    missionWrapper->misi[MISI_SPEEDCHAL]->setP1(ui->doubleSpinBoxP1Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setI1(ui->doubleSpinBoxI1Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setD1(ui->doubleSpinBoxD1Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setP2(ui->doubleSpinBoxP2Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setI2(ui->doubleSpinBoxI2Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setD2(ui->doubleSpinBoxD2Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setP3(ui->doubleSpinBoxP3Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setI3(ui->doubleSpinBoxI3Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setD3(ui->doubleSpinBoxD3Misi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setP4(ui->doubleSpinBoxPDistance_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setI4(ui->doubleSpinBoxIDistance_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setD4(ui->doubleSpinBoxDDistance_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setDD(ui->doubleSpinBoxDDistance->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setServoMuterMax(ui->spinBoxServoMuter->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setYThreshold(ui->spinBoxYThresholdMisi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setSpeed(ui->spinBoxSpeedMisi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setTimeout(ui->breakTimeMisi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setBreakWPIndex(ui->breakWaypointIndexMisi_2->value());
    missionWrapper->misi[MISI_SPEEDCHAL]->setUseCamera(ui->breakUseCameraMisi_2->isChecked());

    missionWrapper->misi[MISI_AUTODOCK]->setStatus(ui->checkBoxMisi_3->isChecked());
    missionWrapper->misi[MISI_AUTODOCK]->setP1(ui->doubleSpinBoxP1Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setI1(ui->doubleSpinBoxI1Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setD1(ui->doubleSpinBoxD1Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setP2(ui->doubleSpinBoxP2Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setI2(ui->doubleSpinBoxI2Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setD2(ui->doubleSpinBoxD2Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setP3(ui->doubleSpinBoxP3Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setI3(ui->doubleSpinBoxI3Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setD3(ui->doubleSpinBoxD3Misi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_AUTODOCK]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_AUTODOCK]->setDD(ui->doubleSpinBoxDDistance->value());
    missionWrapper->misi[MISI_AUTODOCK]->setSpeed(ui->spinBoxSpeedMisi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setSpeedSlow(ui->spinBoxSpeedSlowMisi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setLidarControlRange(ui->spinBoxLidarControlRangeMisi_3->value());
    // missionWrapper->misi[MISI_AUTODOCK]->setThreshold(ui->spinBoxThresholdPinger->value());
    // missionWrapper->misi[MISI_AUTODOCK]->setMaxPinger(ui->spinBoxMaxPinger->value());
    // missionWrapper->misi[MISI_AUTODOCK]->setFrequency(ui->spinBoxThresholdFrequency->value());
    missionWrapper->misi[MISI_AUTODOCK]->setTimeout(ui->breakTimeMisi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setBreakWPIndex(ui->breakWaypointIndexMisi_3->value());
    missionWrapper->misi[MISI_AUTODOCK]->setUseCamera(ui->breakUseCameraMisi_3->isChecked());
    missionWrapper->misi[MISI_AUTODOCK]->setUseSomething(ui->checkBoxUseHydrophone->isChecked());

    missionWrapper->misi[MISI_RAISETHEFLAG]->setStatus(ui->checkBoxMisi_4->isChecked());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setP1(ui->doubleSpinBoxP1Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setI1(ui->doubleSpinBoxI1Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setD1(ui->doubleSpinBoxD1Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setP2(ui->doubleSpinBoxP2Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setI2(ui->doubleSpinBoxI2Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setD2(ui->doubleSpinBoxD2Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setP3(ui->doubleSpinBoxP3Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setI3(ui->doubleSpinBoxI3Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setD3(ui->doubleSpinBoxD3Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setP4(ui->doubleSpinBoxP4Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setI4(ui->doubleSpinBoxI4Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setD4(ui->doubleSpinBoxD4Misi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setDD(ui->doubleSpinBoxDDistance->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setSpeed(ui->spinBoxSpeedMisi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setSpeedSlow(ui->spinBoxSpeedSlowMisi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setTimeout(ui->breakTimeMisi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setBreakWPIndex(ui->breakWaypointIndexMisi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setUseCamera(ui->breakUseCameraMisi_4->isChecked());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setLidarControlRange(ui->spinBoxLidarControlRangeMisi_4->value());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setUseSomething(ui->checkBoxUseDrone->isChecked());
    missionWrapper->misi[MISI_RAISETHEFLAG]->setUseSomething2(ui->checkBoxUseLidar->isChecked());
    missionWrapper->misi[MISI_RAISETHEFLAG]->drone->getHomeLatitude(ui->droneHomeLatitude->text());
    missionWrapper->misi[MISI_RAISETHEFLAG]->drone->getHomeLongitude(ui->droneHomeLongitude->text());

    missionWrapper->misi[MISI_FINDPATH]->setStatus(ui->checkBoxMisi_5->isChecked());
    missionWrapper->misi[MISI_FINDPATH]->setP1(ui->doubleSpinBoxP1Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setI1(ui->doubleSpinBoxI1Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setD1(ui->doubleSpinBoxD1Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setP2(ui->doubleSpinBoxP2Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setI2(ui->doubleSpinBoxI2Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setD2(ui->doubleSpinBoxD2Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setP3(ui->doubleSpinBoxP3Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setI3(ui->doubleSpinBoxI3Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setD3(ui->doubleSpinBoxD3Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setP4(ui->doubleSpinBoxP4Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setI4(ui->doubleSpinBoxI4Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setD4(ui->doubleSpinBoxD4Misi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_FINDPATH]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_FINDPATH]->setDD(ui->doubleSpinBoxDDistance->value());
    missionWrapper->misi[MISI_FINDPATH]->setYThreshold(ui->spinBoxYThresholdMisi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setServoMuterMax(ui->spinBoxServoMuter_2->value());
    missionWrapper->misi[MISI_FINDPATH]->setSpeed(ui->spinBoxSpeedMisi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setSpeedSlow(ui->spinBoxSpeedSlowMisi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setTimeout(ui->breakTimeMisi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setBreakWPIndex(ui->breakWaypointIndexMisi_5->value());
    missionWrapper->misi[MISI_FINDPATH]->setUseCamera(ui->breakUseCameraMisi_5->isChecked());

    missionWrapper->misi[MISI_RETURNHOME]->setP1(ui->doubleSpinBoxP1Misi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setI1(ui->doubleSpinBoxI1Misi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setD1(ui->doubleSpinBoxD1Misi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setP2(ui->doubleSpinBoxP1Misi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setI2(ui->doubleSpinBoxI1Misi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setD2(ui->doubleSpinBoxD1Misi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_RETURNHOME]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_RETURNHOME]->setDD(ui->doubleSpinBoxDDistance->value());
    missionWrapper->misi[MISI_RETURNHOME]->setSpeed(ui->spinBoxSpeedMisi_7->value());
    missionWrapper->misi[MISI_RETURNHOME]->setPD(ui->doubleSpinBoxPDistance->value());
    missionWrapper->misi[MISI_RETURNHOME]->setID(ui->doubleSpinBoxIDistance->value());
    missionWrapper->misi[MISI_RETURNHOME]->setDD(ui->doubleSpinBoxDDistance->value());

    missionWrapper->misi[MISI_FOLLOWLEADER]->setStatus(ui->checkBoxMisi_6->isChecked());
}

PositionType MainWindow::calculateCoordinate(const QPoint &pt){
    PositionType pos;
    pos.latitude     = (pt.y()/ skala_latitude  + latitude_awal );
    pos.longitude    = (pt.x() / skala_longitude + longitude_awal );
    return pos;
}

QPoint MainWindow::calculatePoint(const PositionType& pos){
    QPoint pt;
    pt.setX( (pos.longitude - longitude_awal) * skala_longitude );
    pt.setY( (pos.latitude - latitude_awal) * skala_latitude );
    return pt;
}

QPoint MainWindow::calculatePoint(double lat, double lon){
    QPoint pt;
    pt.setX( (lon - longitude_awal) * skala_longitude );
    pt.setY( (lat - latitude_awal) * skala_latitude );
    return pt;
}


void MainWindow::changeMissionSequence(){
    missionWrapper->changeMissionSequence(ui->lineEditMissionSequence->text());
}

void MainWindow::displayCurrentMission(int misiIdx){
    ui->lineEditCurrentMission->setText(missionWrapper->misi[misiIdx]->getName());
}

void MainWindow::displayCurrentState(QString state){
    ui->lineEditCurrentMission_2->setText(state);
}

void MainWindow::switchMissionTab(int misiIdx){
    ui->tabWidgetMisi->setCurrentIndex(misiIdx);
}

void MainWindow::clearOutputs(){
    ui->lineEditCurrentMission->clear();
    ui->lineEditErrorSudut->clear();
    ui->lineEditSudutTujuan->clear();
    ui->lineEditJarak->clear();
    ui->lcdNumberWaypoint->display(0);
    ui->lineEditCurrentMission_2->clear();
    ui->lineEditOutput->clear();
}

void MainWindow::receiveFlagColor(int color){
    if(color == RED_FLAG){
        ui->labelFlagColor->setText("RED");
        ui->labelFlagColor->setStyleSheet("color: red");
    }
    else{
        ui->labelFlagColor->setText("BLUE");
        ui->labelFlagColor->setStyleSheet("color: blue");
    }
}

void MainWindow::imageReceive(QImage* image){
    if(image->width()){
        QPainter painter(image);
        painter.setPen(k);
        painter.drawLine(topLeftX, topLeftY, botLeftX, botLeftY);
        painter.drawLine(topRightX, topRightY, botRightX, botRightY);
        QPoint _ = missionWrapper->getCameraControlPoint();
        missionWrapper->resetCameraControlPoint();
        if(_!=controlPoint){
            controlPoint = _;
        }
        else{
            controlPoint.setX(-6);
            controlPoint.setY(-6);
        }
        painter.setPen(r);
        painter.drawLine(controlPoint.x(), controlPoint.y()-15, controlPoint.x(), controlPoint.y()+15);
        painter.setPen(b);
        painter.drawLine(0, yThreshold, image->width(), yThreshold);
        painter.end();
    }
    if(ui->tabWidget->currentIndex() == 0){
        ui->labelOri->setPixmap(QPixmap::fromImage(*image));
    }
    else if(ui->tabWidget->currentIndex() == 2){
        ui->labelCameraSmall->setPixmap(QPixmap::fromImage(image->scaled(480,360,Qt::KeepAspectRatio)));
    }
    delete image;
//    qDebug()<<"Image received";
}

void MainWindow::showInfoMap(QPoint& pt){
    PositionType pos = calculateCoordinate(pt);
    QToolTip::showText(cursor().pos(),
                       "x: " + QString::number(pt.x())
                        + " | y: " + QString::number(pt.y())
                        + "\nLatitude : " + QString::number(pos.latitude, 'g', 10)
                        + "\nLongitude: " + QString::number(pos.longitude, 'g', 10),
                       ui->labelMapping
                       );
}

void MainWindow::setWaypointFromMission(double latitude, double longitude, int idx){
    if(idx<0)return;
    PositionType temp;
    temp.latitude = latitude;
    temp.longitude = longitude;
    unsigned misiIndex = ui->tabWidgetMisi->currentIndex();
    if(misiIndex == 0) return;
    comboBoxWaypoint[misiIndex]->setValue(idx);
    QPoint _ = calculatePoint(temp);
    setWaypointAndNext(_);
}

void MainWindow::setWaypoint(QPoint& pt){
    ui->labelMapping->normalizePosition(pt);
    PositionType pos = calculateCoordinate(pt);
    unsigned misiIndex = ui->tabWidgetMisi->currentIndex();

    if(misiIndex == 0)
        return;

    unsigned index      = comboBoxWaypoint[misiIndex]->value();
    QString latitude    = QString::number(pos.latitude, 'g', 10);
    QString longitude   = QString::number(pos.longitude, 'g', 10);

    tableWidgetWaypoint[misiIndex]->setItem(index,
                                     0,
                                     new QTableWidgetItem(latitude)
                                    );

    tableWidgetWaypoint[misiIndex]->setItem(index,
                                     1,
                                     new QTableWidgetItem(longitude)
                                    );
    setPositionField(pt);
    emit waypointChanged(misiIndex, index, pos);

    MapPoint mp;
    mp.pt = pt;
    mp.labeltext = ui->tabWidgetMisi->tabText(misiIndex) + " : " + QString::number(comboBoxWaypoint[misiIndex]->value());
    emit waypointChanged(misiIndex, index, mp);
}

void MainWindow::inputWaypoint(){
    qDebug() << "input waypoint";
    QPoint pt;
    PositionType pos;
    unsigned misiIndex = ui->tabWidgetMisi->currentIndex();

    if (misiIndex == 0)
        return;

    unsigned index = comboBoxWaypoint[misiIndex]->value();

    pos.latitude  = ui->lineEditInputLatt->text().toDouble();
    pos.longitude = ui->lineEditInputLong->text().toDouble();

    pt = calculatePoint(pos);

    tableWidgetWaypoint[misiIndex]->setItem(index,
                                     0,
                                     new QTableWidgetItem(ui->lineEditInputLatt->text())
                                    );
    tableWidgetWaypoint[misiIndex]->setItem(index,
                                     1,
                                     new QTableWidgetItem(ui->lineEditInputLong->text())
                                    );

    emit waypointChanged(misiIndex, index, pos);

    MapPoint mp;
    mp.pt = pt;
    mp.labeltext = ui->tabWidgetMisi->tabText(misiIndex) + " : " + QString::number(comboBoxWaypoint[misiIndex]->value());
    emit waypointChanged(misiIndex, index, mp);
}

void MainWindow::setWaypointAndNext(QPoint& pt){
    setWaypoint(pt);
    unsigned misiIndex = ui->tabWidgetMisi->currentIndex();

    if(misiIndex == 0) return;

    unsigned index = comboBoxWaypoint[misiIndex]->value();

    //qDebug() << comboBoxWaypoint[misiIndex]->maximum() << " " << index;
    if (index+1 > comboBoxWaypoint[misiIndex]->maximum()){
            addWaypoint(misiIndex);
    }
    comboBoxWaypoint[misiIndex]->setValue( index + 1 );
}

void MainWindow::addWaypoint(unsigned misiIndex){
    comboBoxWaypoint[misiIndex]->setMaximum(comboBoxWaypoint[misiIndex]->maximum() + 1);
    tableWidgetWaypoint[misiIndex]->setRowCount(tableWidgetWaypoint[misiIndex]->rowCount() + 1);
}

void MainWindow::setCurrPosAsWaypoint(){
    setWaypointAndNext(ui->labelMapping->getInitCircle());
}

void MainWindow::resetAllWaypoints(){
    qDebug() << "reset all waypoints";
    for(unsigned index=1; index<NUM_MISI; index++){
        resetWaypoints(index);
    }
}

void MainWindow::resetWaypoints(){
    unsigned misiIndex = ui->tabWidgetMisi->currentIndex();

    if(misiIndex==0)
        return;

    resetWaypoints(ui->tabWidgetMisi->currentIndex());
}

void MainWindow::resetWaypoints(unsigned misiIndex){
    tableWidgetWaypoint[misiIndex]->clear();
    tableWidgetWaypoint[misiIndex]->setRowCount(1);
    comboBoxWaypoint[misiIndex]->setValue(0);
    comboBoxWaypoint[misiIndex]->setMaximum(0);
    ui->lineEditInputLatt->clear();
    ui->lineEditInputLong->clear();
    missionWrapper->misi[misiIndex]->clearWaypoints();
}

void MainWindow::deleteWaypoint(){
    unsigned misiIdx = ui->tabWidgetMisi->currentIndex();
    int wpIdx = comboBoxWaypoint[misiIdx]->value();
    if(wpIdx>=0){
        if(tableWidgetWaypoint[misiIdx]->item(wpIdx,0)){
            if(misiIdx == 8)
                missionWrapper->misi[MISI_RAISETHEFLAG]->deleteDroneWaypoint(wpIdx);
            else
                missionWrapper->misi[misiIdx]->deleteWaypoint(wpIdx);
            ui->labelMapping->deleteCircle(misiIdx, wpIdx);
            if(wpIdx == tableWidgetWaypoint[misiIdx]->rowCount()-1){
                tableWidgetWaypoint[misiIdx]->removeRow(wpIdx);
                tableWidgetWaypoint[misiIdx]->insertRow(tableWidgetWaypoint[misiIdx]->rowCount());
            }
            else if(wpIdx<tableWidgetWaypoint[misiIdx]->rowCount()-1){
                tableWidgetWaypoint[misiIdx]->removeRow(wpIdx);
                tableWidgetWaypoint[misiIdx]->insertRow(tableWidgetWaypoint[misiIdx]->rowCount());
                comboBoxWaypoint[misiIdx]->setMaximum(comboBoxWaypoint[misiIdx]->maximum() - 1);
                tableWidgetWaypoint[misiIdx]->setRowCount(tableWidgetWaypoint[misiIdx]->rowCount() - 1);
            }
        }
        if(wpIdx>0){
            comboBoxWaypoint[misiIdx]->setValue(wpIdx-1);
        }
    }
}

void MainWindow::deleteWaypointFromMission(int idx){
    unsigned misiIdx = ui->tabWidgetMisi->currentIndex();
    comboBoxWaypoint[misiIdx]->setValue(idx);
    deleteWaypoint();
}

void MainWindow::setPositionField(QPoint& pt){
    PositionType pos = calculateCoordinate(pt);
    ui->lineEditInputLatt->setText(QString::number(pos.latitude, 'g', 10));
    ui->lineEditInputLong->setText(QString::number(pos.longitude, 'g', 10));
}

void MainWindow::onChangeNextDest(int nextDest, const PositionType* pos){
    ui->lcdNumberWaypoint->display(nextDest);
    ui->labelMapping->setNextCircle(nextDest);
    ui->labelMapping->setStartPos((pos->latitude - latitude_awal) * skala_latitude, (pos->longitude - longitude_awal) * skala_longitude);
}

void MainWindow::onUpdateSudutJarak(double sudut, double error, double jarak){
    ui->lineEditSudutTujuan->setText(QString::number(sudut, 'g', 3));
    ui->lineEditErrorSudut->setText(QString::number(error, 'g', 3));
    ui->lineEditJarak->setText(QString::number(jarak, 'g', 3));
}

void MainWindow::changeCurrentPos(double latitude, double longitude, double compass){
    tempPos.latitude = latitude;
    tempPos.longitude = longitude;
    QPoint pt = calculatePoint(tempPos);
    // qDebug() << "MainWindow: currentPos changed";

    emit currentPosChanged(pt);
    emit newAngle(compass);
}

void MainWindow::openSensor(){
    ui->pushButtonOpenSensor->setDisabled(true);
    ui->pushButtonCloseSensor->setEnabled(true);
    clock_t startTime;
    FILE *stream;
    stream = popen("ls /dev/ttyACM*", "r");
    char b[30];
    while (!feof(stream)){
        if (fgets(b, 30, stream) != NULL){
            b[strlen(b)-1]=0;
            portNames.push_back(std::string(b));
        }
    }
    // Connecting Pixhawk 29,1530678  -81,017243
    if(ui->checkGPS->isChecked()){
        emit startGPS(&portNames);
        // Waiting Pixhawk
        while(gpsConnection->getPortNum()==-1)usleep(100);
        if(gpsConnection->getPortNum()>-1)portNames.erase(portNames.begin()+gpsConnection->getPortNum());
    }
    // Connecting Smart Phone
    if(ui->checkGPS_Phone->isChecked()){
        emit startPhoneGPS();
    }
    // Connecting SRF
    if(ui->checkSRF->isChecked()){
        emit startSRF(&portNames);
        // Waiting SRF
        while(srf->getPortNum()==-1)usleep(100);
        if(srf->getPortNum()>-1)portNames.erase(portNames.begin()+srf->getPortNum());
    }
    portNames.clear();
    pclose(stream);
    stream = popen("ls /dev/ttyUSB*", "r");
    while (!feof(stream)){
        if (fgets(b, 30, stream) != NULL){
            b[strlen(b)-1]=0;
            portNames.push_back(std::string(b));
        }
    }
    // Connecting Lidar
    if(0){
        emit startLidar(&portNames);
        // Waiting Lidar
        while(lidar->getPortNum()==-1)usleep(100);
        if(lidar->getPortNum()>-1)portNames.erase(portNames.begin()+lidar->getPortNum());
    }
    // Connecting STM
    missionWrapper->connectToSTM(&portNames);
    portNames.clear();
    pclose(stream);
}

void MainWindow::closeSensor(){
    emit stopGPS();
    emit stopLidar();
    emit stopSRF();
    tempPos.latitude = tempPos.longitude = NAN;
    ui->pushButtonOpenSensor->setEnabled(true);
    ui->pushButtonCloseSensor->setDisabled(true);
}

void MainWindow::stopCamThread(){
    emit killImgReceiver();
    if(camThread->isRunning()) camThread->quit();
    camThread->deleteLater();
    qDebug() << "cam thread quit";
}

void::MainWindow::stopMissionWrapperThread(){
    if(missionWrapperThread->isRunning()) missionWrapperThread->quit();
    missionWrapperThread->deleteLater();
    qDebug() << "misison wrapper thread quit";
}

void MainWindow::stopTCPThread(){
    if(tcpThread->isRunning()) tcpThread->quit();
    tcpThread->deleteLater();
    qDebug() << "tcp thread quit";
}

void ::MainWindow::stopGPSThread(){
    emit stopGPS();
    if(gpsThread->isRunning()) gpsThread->quit();
    gpsThread->deleteLater();
    qDebug() << "gps thread quit";
}

void ::MainWindow::stopSRFThread(){
    emit stopSRF();
    if(srfThread->isRunning()) srfThread->quit();
    srfThread->deleteLater();
    qDebug() << "srf thread quit";
}

void MainWindow::onSave(){
    QString filename;
    filename = QFileDialog::getSaveFileName(this,
                                            "Save Waypoints",                           // caption
                                            "Data/",                                    // directory
                                            "JSON (*.json)"                             // options
                                            );
    if (filename.isEmpty()) {
            return;
    }
    else{
        QFile *file = new QFile(filename);

        // cek file mode readwrite dan mode truncate (bukan append)
        if(!file->open(QIODevice::ReadWrite | QIODevice::Truncate)){
            QMessageBox::information(this, tr("Tidak bisa membuka file"), file->errorString());
        }
        QJsonObject json;

        QJsonArray thresholdArray;
        for(int i=0; i<NUM_COLORS; i++){
            ThresholdWidget *tw = thresbox[i];
            QJsonObject thresholdObject;
            tw->write(thresholdObject);
            thresholdArray.append(thresholdObject);
        }

        QJsonObject misiObject;
        missionWrapper->write(misiObject);

        json["hsv"] = thresholdArray;
        json["missionSeq"] = ui->lineEditMissionSequence->text();
        json["missions"] = misiObject;

        QJsonDocument saveDoc(json);
        file->write(saveDoc.toJson());

        file->close();
        delete file;
    }
}

void MainWindow::onLoad(){
    QString filename;
    QString buffer;
    filename = QFileDialog::getOpenFileName(this,
                                            "Load Waypoint",
                                            "Data/",
                                            "JSON(*.json)"
                                            );
    if (filename .isEmpty()) {
        return;
    }
    else {
        QFile* file = new QFile(filename);
        if (!file->open(QIODevice::ReadWrite)) {
            QMessageBox::warning(this, tr("Tidak bisa memuat file."), file->errorString());
            return;
        }

        QByteArray saveData = file->readAll();
        QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));
        QJsonObject obj = loadDoc.object();

        QJsonArray thresholdArray = obj["hsv"].toArray();
        for(int i=0; i<thresholdArray.size(); i++){
            thresbox[i]->load(thresholdArray[i].toObject());
        }

        missionsAttributeLoaded(obj);

        file->close();
        delete file;
    }
}

void MainWindow::missionsAttributeLoaded(QJsonObject &obj){
    QJsonArray misiArr = obj["missions"].toObject()["misi"].toArray();
    QJsonObject mo;
    QJsonObject pid;

    // misi 1
    mo = misiArr[0].toObject();
    pid = mo["pid"].toArray()[0].toObject();
    ui->doubleSpinBoxP1Misi_1->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_1->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_1->setValue(pid["d"].toDouble()); usleep(2000);

    pid = mo["pid"].toArray()[1].toObject();
    ui->doubleSpinBoxP2Misi_1->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI2Misi_1->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD2Misi_1->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_1->setValue(mo["speed"].toInt()); usleep(2000);
    ui->spinBoxSudutMisi_1->setValue(mo["sudut"].toInt()); usleep(2000);

    ui->breakTimeMisi_1->setValue(mo["timeout"].toDouble()); usleep(2000);
    ui->breakWaypointIndexMisi_1->setValue(mo["breakwp"].toInt()); usleep(2000);
    ui->breakUseCameraMisi_1->setChecked(mo["useCam"].toInt()); usleep(2000);

    // misi 2
    mo = misiArr[1].toObject();
    pid = mo["pid"].toArray()[0].toObject();
    ui->doubleSpinBoxP1Misi_2->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_2->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_2->setValue(pid["d"].toDouble()); usleep(2000);

    pid = mo["pid"].toArray()[1].toObject();
    ui->doubleSpinBoxP2Misi_2->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI2Misi_2->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD2Misi_2->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_2->setValue(mo["speed"].toInt()); usleep(2000);

    ui->breakTimeMisi_2->setValue(mo["timeout"].toDouble()); usleep(2000);
    ui->breakWaypointIndexMisi_2->setValue(mo["breakwp"].toInt()); usleep(2000);
    ui->breakUseCameraMisi_2->setChecked(mo["useCam"].toInt()); usleep(2000);

    // misi 3
    mo = misiArr[2].toObject();
    pid = mo["pid"].toArray()[0].toObject();
    ui->doubleSpinBoxP1Misi_3->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_3->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_3->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_3->setValue(mo["speed"].toInt()); usleep(2000);
    ui->spinBoxSpeedMisi_3->setValue(mo["speed"].toInt()); usleep(2000);
    ui->spinBoxSudutMisi_3->setValue(mo["sudut"].toInt()); usleep(2000);
    ui->spinBoxThresholdPinger->setValue(mo["threshold"].toInt()); usleep(2000);
    ui->spinBoxMaxPinger->setValue(mo["maxPinger"].toInt()); usleep(2000);
    ui->spinBoxThresholdFrequency->setValue(mo["frequency"].toInt()); usleep(2000);

    ui->breakTimeMisi_3->setValue(mo["timeout"].toDouble()); usleep(2000);
    ui->breakWaypointIndexMisi_3->setValue(mo["breakwp"].toInt()); usleep(2000);
    ui->breakUseCameraMisi_3->setChecked(mo["useCam"].toInt()); usleep(2000);

    // misi 4
    mo = misiArr[3].toObject();
    pid = mo["pid"].toArray()[0].toObject(); usleep(2000);
    ui->doubleSpinBoxP1Misi_4->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_4->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_4->setValue(pid["d"].toDouble()); usleep(2000);

    pid = mo["pid"].toArray()[1].toObject();
    ui->doubleSpinBoxP2Misi_4->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI2Misi_4->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD2Misi_4->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_4->setValue(mo["speed"].toInt()); usleep(2000);
    ui->spinBoxSpeedSlowMisi_4->setValue(mo["speed2"].toInt()); usleep(2000);

    ui->breakTimeMisi_4->setValue(mo["timeout"].toDouble()); usleep(2000);
    ui->breakWaypointIndexMisi_4->setValue(mo["breakwp"].toInt()); usleep(2000);
    ui->breakUseCameraMisi_4->setChecked(mo["useCam"].toInt()); usleep(2000);

    // misi 5
    mo = misiArr[4].toObject();
    pid = mo["pid"].toArray()[0].toObject();
    ui->doubleSpinBoxP1Misi_5->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_5->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_5->setValue(pid["d"].toDouble()); usleep(2000);

    pid = mo["pid"].toArray()[1].toObject();
    ui->doubleSpinBoxP2Misi_5->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI2Misi_5->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD2Misi_5->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_5->setValue(mo["speed"].toInt()); usleep(2000);

    ui->breakTimeMisi_5->setValue(mo["timeout"].toDouble()); usleep(2000);
    ui->breakWaypointIndexMisi_5->setValue(mo["breakwp"].toInt()); usleep(2000);
    ui->breakUseCameraMisi_5->setChecked(mo["useCam"].toInt()); usleep(2000);

    // misi 6
    mo = misiArr[5].toObject();
    pid = mo["pid"].toArray()[0].toObject();
    ui->doubleSpinBoxP1Misi_6->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_6->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_6->setValue(pid["d"].toDouble()); usleep(2000);

    pid = mo["pid"].toArray()[1].toObject();
    ui->doubleSpinBoxP2Misi_6->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI2Misi_6->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD2Misi_6->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_6->setValue(mo["speed"].toInt()); usleep(2000);
    ui->spinBoxSpeedBawahMisi_6->setValue(mo["sudut"].toInt()); usleep(2000);

    ui->breakTimeMisi_6->setValue(mo["timeout"].toDouble()); usleep(2000);
    ui->breakWaypointIndexMisi_6->setValue(mo["breakwp"].toInt()); usleep(2000);
    ui->breakUseCameraMisi_6->setChecked(mo["useCam"].toInt()); usleep(2000);

    // misi 7
    mo = misiArr[6].toObject();
    pid = mo["pid"].toArray()[0].toObject();
    ui->doubleSpinBoxP1Misi_7->setValue(pid["p"].toDouble()); usleep(2000);
    ui->doubleSpinBoxI1Misi_7->setValue(pid["i"].toDouble()); usleep(2000);
    ui->doubleSpinBoxD1Misi_7->setValue(pid["d"].toDouble()); usleep(2000);

    ui->spinBoxSpeedMisi_7->setValue(mo["speed"].toInt()); usleep(2000);

}

void MainWindow::onSaveAsWaypoints(){
    QString filename;
    filename = QFileDialog::getSaveFileName(this,
                                            "Save Waypoints",                           // caption
                                            "Data/",                                    // directory
                                            "CSV (*.csv);;Text (*.txt);;ALL File(*.)"  	// options
                                            );
    if (filename.isEmpty()) {
            return;
    }
    else{
        QFile *file = new QFile(filename);

        // cek file mode readwrite dan mode truncate (bukan append)
        if(!file->open(QIODevice::ReadWrite | QIODevice::Truncate)){
            QMessageBox::information(this, tr("Tidak bisa membuka file"), file->errorString());
        }
        QTextStream out(file);
        for (unsigned i = 1; i < NUM_MISI; i++) {
            for(unsigned index=0; index<missionWrapper->misi[i]->NUM_WP; index++){
                PositionType waypoint = missionWrapper->getWaypoint(i, index);
                out << QString::number(waypoint.latitude, 'g', 10)
                    << ','
                    << QString::number(waypoint.longitude, 'g', 10)
                    << endl;
            }
            out << "--" << endl;
        }
        file->close();
        delete file;
        this->filenameWaypoint = filename;

    }
}

void MainWindow::onSaveWaypoints(){

}

void MainWindow::onLoadWaypoints(){
    QString filename;
    QString buffer;
    filename = QFileDialog::getOpenFileName(this,
                                            "Load Waypoint",
                                            "Data/",
                                            "CSV (*.csv);;Text (*.txt);;ALL File(*.)"
                                            );
    if (filename .isEmpty()) {
        return;
    }
    else {
        resetAllWaypoints();
        QFile* file = new QFile(filename);
        if (!file->open(QIODevice::ReadWrite)) {
            QMessageBox::warning(this, tr("Tidak bisa memuat file."), file->errorString());
            return;
        }

        QTextStream in(file);
        QTableWidgetItem *latt, *longi;
        qDebug() << "Masuk load waypoint";
        QStringList  allReaded = in.readAll().split("\n");
        qDebug() << allReaded.length() << allReaded.size();
        unsigned misiIdx = 1;
        int wpIndex = 0;
        for (unsigned index = 0; index < allReaded.length() - 1; index++) {
                buffer = allReaded.at(index);

                // jika kebaca batasan misi
                if(buffer == "--"){
                    misiIdx++;
                    wpIndex = 0;
                    continue;
                }

                qDebug() << "Buffer: " << buffer;
                QStringList wordList = buffer.split(",");
                PositionType pos;
                pos.latitude  = wordList[0].toDouble();
                pos.longitude = wordList[1].toDouble();
                emit waypointLoaded(misiIdx, wpIndex++, pos);
        }
        file->close();
        delete file;
        this->filenameWaypoint = filename;
    }
}

void MainWindow::loadWaypoint(unsigned misiIndex, unsigned index, PositionType& pos){
//    set data waypoint kontrol kapal
    emit waypointChanged(misiIndex, index, pos);
    if(pos.isNull())
        return;

//    gambar waypoint labelMapping
    MapPoint pt;
    pt.pt = calculatePoint(pos);
    pt.labeltext = ui->tabWidgetMisi->tabText(misiIndex) + " : " + QString::number(index);
    emit waypointChanged(misiIndex, index, pt);
//    set item tabel waypoint
    addWaypoint(misiIndex);
    qDebug() << misiIndex;
    tableWidgetWaypoint[misiIndex]->setItem(index,  0,
                                     new QTableWidgetItem(QString::number(pos.latitude, 'g', 10))
                                     );
    tableWidgetWaypoint[misiIndex]->setItem(index, 1,
                                     new QTableWidgetItem(QString::number(pos.longitude, 'g', 10))
                                     );
}

void MainWindow::onStartCam(){
    ui->startCamButton->setDisabled(true);
    ui->stopCamButton->setEnabled(true);
    ui->checkBoxThreshold->setDisabled(true);
}

void MainWindow::onStopCam(){
    ui->stopCamButton->setDisabled(true);
    ui->startCamButton->setEnabled(true);
    ui->labelOri->clear();
    ui->labelBin->clear();
    ui->labelCameraSmall->clear();
    ui->checkBoxThreshold->setEnabled(true);
}

void MainWindow::setDanau8(){
    ui->pushButtonLocationDanau8->setDisabled(true);
    ui->pushButtonLocationAlpha->setEnabled(true);
    ui->pushButtonLocationBravo->setEnabled(true);
    ui->pushButtonLocationCharlie->setEnabled(true);
    ui->pushButtonLocationDelta->setEnabled(true);
    ui->pushButtonLocationRobotika->setEnabled(true);
    setLocation(0, ":images/danau8.jpg");
}

void MainWindow::setAlpha(){
    ui->pushButtonLocationDanau8->setEnabled(true);
    ui->pushButtonLocationAlpha->setDisabled(true);
    ui->pushButtonLocationBravo->setEnabled(true);
    ui->pushButtonLocationCharlie->setEnabled(true);
    ui->pushButtonLocationDelta->setEnabled(true);
    ui->pushButtonLocationRobotika->setEnabled(true);
    setLocation(1, ":images/alpha.png");
}

void MainWindow::setBravo(){
    ui->pushButtonLocationDanau8->setEnabled(true);
    ui->pushButtonLocationAlpha->setEnabled(true);
    ui->pushButtonLocationBravo->setDisabled(true);
    ui->pushButtonLocationCharlie->setEnabled(true);
    ui->pushButtonLocationDelta->setEnabled(true);
    ui->pushButtonLocationRobotika->setEnabled(true);
    setLocation(2, ":images/bravo.png");
}

void MainWindow::setCharlie(){
    ui->pushButtonLocationDanau8->setEnabled(true);
    ui->pushButtonLocationAlpha->setEnabled(true);
    ui->pushButtonLocationBravo->setEnabled(true);
    ui->pushButtonLocationCharlie->setDisabled(true);
    ui->pushButtonLocationDelta->setEnabled(true);
    ui->pushButtonLocationRobotika->setEnabled(true);
    setLocation(3, ":images/charlie.png");
}

void MainWindow::setDelta(){
    ui->pushButtonLocationDanau8->setEnabled(true);
    ui->pushButtonLocationAlpha->setEnabled(true);
    ui->pushButtonLocationBravo->setEnabled(true);
    ui->pushButtonLocationCharlie->setEnabled(true);
    ui->pushButtonLocationDelta->setDisabled(true);
    ui->pushButtonLocationRobotika->setEnabled(true);
    setLocation(4, ":images/delta.png");
}

void MainWindow::setRobotika(){
    ui->pushButtonLocationDanau8->setEnabled(true);
    ui->pushButtonLocationAlpha->setEnabled(true);
    ui->pushButtonLocationBravo->setEnabled(true);
    ui->pushButtonLocationCharlie->setEnabled(true);
    ui->pushButtonLocationDelta->setEnabled(true);
    ui->pushButtonLocationRobotika->setDisabled(true);
    setLocation(5, ":images/danau8.jpg");
}

void MainWindow::setLocation(int course, const char *map_path){
    latitude_awal   = latitude_awal_[course];
    latitude_akhir  = latitude_akhir_[course];
    longitude_awal  = longitude_awal_[course];
    longitude_akhir = longitude_akhir_[course];
    skala_latitude  = (double) 860 / (latitude_akhir - latitude_awal);
    skala_longitude = (double) 960 / (longitude_akhir - longitude_awal);
    boxLat = (latitude_akhir - latitude_awal)/43*11.0*10000.0;
    boxLon = (longitude_akhir - longitude_awal)/48*11.0*10000.0;
    qDebug()<<boxLat<<boxLon;
    emit sendLocation(latitude_awal, longitude_awal, skala_latitude, skala_longitude);
    emit sendGMapPath(map_path);
    emit sendCourseIdx(course);
}

void MainWindow::useThreshold(int state){
    imgReceiver->isThreshold = state;
    imgReceiver->currentColor = ui->groupBoxThres->currentIndex();
}


void MainWindow::binaryImageReceive(QImage* bin){
    ui->labelBin->setPixmap(QPixmap::fromImage(*bin));
    delete bin;
}

void MainWindow::mirrorControlLine(int side){
    // default: 0 == mirror from left line
    if(side==0){
        topRightX = 640-topLeftX; topRightY = topLeftY;
        botRightX = 640-botLeftX; botRightY = botLeftY;
    }
    else{
        topLeftX = 640-topRightX; topLeftY = topRightY;
        botLeftX = 640-botRightX; botLeftY = botRightY;
    }
}

void MainWindow::openTCPConnection(){
    char ip[19];
    char port_[5];
    int port;
    int i;
    memcpy(ip, ui->serverIP->text().toStdString().c_str(), ui->serverIP->text().size());
    ip[ui->serverIP->text().size()] = '\0';
    memcpy(port_, ui->serverPort->text().toStdString().c_str(), ui->serverPort->text().size());
    port_[ui->serverPort->text().size()] = '\0';
    port = atoi(port_);
    ui->btnStopConnServer->setEnabled(true);
    ui->btnStartConnServer->setDisabled(true);
    emit startTCP(ip, port);
}

void MainWindow::closeTCPConnection(){
    ui->btnStopConnServer->setDisabled(true);
    ui->btnStartConnServer->setEnabled(true);
    emit stopTCP();
}

void MainWindow::drawAttitude(){
    QPainter painter(ui->attitudeWidget);
    painter.translate(100,100);
    painter.rotate(rollAngle);
    painter.translate(-100,-100);
    painter.drawPixmap(0,0,*attitudeShip);
}

void MainWindow::getAttitude(double roll){
    rollAngle = roll;
}

void MainWindow::startMission(){
    ui->pushButtonStartMission->setDisabled(true);
    ui->pushButtonNextMission->setEnabled(true);
    ui->pushButtonStopMission->setEnabled(true);
    ui->pushButtonPanic->setEnabled(true);
    ui->labelMapping->unvisitAllWaypoints();
}

void MainWindow::endMission(){
    ui->pushButtonStartMission->setEnabled(true);
    ui->pushButtonStopMission->setDisabled(true);
    ui->pushButtonNextMission->setDisabled(true);
    ui->pushButtonPanic->setDisabled(true);
    tcp->stopConnection();
}

void MainWindow::sensor_input_output_inGUI_handler(){
    FILE *stream;
    stream = popen("sensors | grep Core", "r");
    int coreIdx=0;
    while (!feof(stream)){
        int i=0;
        if (fgets(buffer, 70, stream) != NULL){
            for(;i<7;i++)coreTemp[coreIdx][i] = buffer[i+16];
            coreTemp[coreIdx][i] = 0;
            ++coreIdx;
        }
    }
    if(strlen(coreTemp[0])>3)ui->tempCore_0->setText(coreTemp[0]);
    if(strlen(coreTemp[1])>3)ui->tempCore_1->setText(coreTemp[1]);
    if(strlen(coreTemp[2])>3)ui->tempCore_2->setText(coreTemp[2]);
    if(strlen(coreTemp[3])>3)ui->tempCore_3->setText(coreTemp[3]);
    if(strlen(coreTemp[4])>3)ui->tempCore_4->setText(coreTemp[4]);
    if(strlen(coreTemp[5])>3)ui->tempCore_5->setText(coreTemp[5]);
    pclose(stream);
    stream = popen("nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader", "r");
    if(fgets(buffer, 3, stream) != NULL){
        sprintf(gpuTemp, "%c%c°C", buffer[0], buffer[1]);
    }
    if(strlen(gpuTemp)>3)ui->tempGPU->setText(gpuTemp);
    pclose(stream);

    sprintf(tempPosString, "%.8lf", tempPos.latitude);
    ui->lineEditCurrLat->setText(tempPosString);
    sprintf(tempPosString, "%.8lf", tempPos.longitude);
    ui->lineEditCurrLong->setText(tempPosString);
    
    sprintf(srfStringL, "%d", srfData[0]);
    sprintf(srfStringML, "%d", srfData[1]);
    sprintf(srfStringMR, "%d", srfData[2]);
    sprintf(srfStringR, "%d", srfData[3]);
    sprintf(srfStringS, "%d", srfData[4]);
    ui->lineEditSRFKiri->setText(srfStringL);
    ui->lineEditSRFMKiri->setText(srfStringML);
    ui->lineEditSRFMKanan->setText(srfStringMR);
    ui->lineEditSRFKanan->setText(srfStringR);
    ui->lineEditSRFSamping->setText(srfStringS);

    ui->lineEditOutput->setText(missionWrapper->getOutput());
    ui->lineEditControlStatus->setText(missionWrapper->getMissionControl());
}

void MainWindow::setExposure(int x){
    char buff[256];
    sprintf(buff, "v4l2-ctl --set-ctrl=exposure_auto=0 --set-ctrl=exposure_absolute=%d", x);
    FILE *stream = popen(buff, "r");
    pclose(stream);
    qDebug()<< "exposure set";
}


void MainWindow::setGain(int x){
    char buff[256];
    sprintf(buff, "v4l2-ctl --set-ctrl=gain=%d", x);
    FILE *stream = popen(buff, "r");
    pclose(stream);
    
    qDebug()<< "exposure set";
}


void MainWindow::setWhiteBalance(int x){
    char buff[256];
    sprintf(buff, "v4l2-ctl --set-ctrl=white_balance_temperature_auto=0 --set-ctrl=white_balance_temperature=%d", x);
    FILE *stream = popen(buff, "r");
    pclose(stream);
    
    qDebug()<< "exposure set";
}

void MainWindow::runTest(){
    ui->startTest->setDisabled(true);
    ui->stopTest->setEnabled(true);
}

void MainWindow::stopTest(){
    ui->stopTest->setDisabled(true);
    ui->startTest->setEnabled(true);
}

void MainWindow::setControlLine(bool left, int topX, int topY, int botX, int botY){
    if(left){
        topLeftX = topX; topLeftY = topY; botLeftX = botX; botLeftY = botY;
        mirrorControlLine();
    }
    else{
        topRightX = topX; topRightY = topY; botRightX = botX; botRightY = botY;
        mirrorControlLine(1);
    }
}

void MainWindow::setYThreshold(int val){
    yThreshold = val;
}

void MainWindow::forward(){
    ui->spinBoxMotorKiriTester->setValue(ui->spinBoxMotorKiriTester->value()+10);
    ui->spinBoxMotorKananTester->setValue(ui->spinBoxMotorKananTester->value()+10);
}

void MainWindow::backward(){
    ui->spinBoxMotorKiriTester->setValue(ui->spinBoxMotorKiriTester->value()-10);
    ui->spinBoxMotorKananTester->setValue(ui->spinBoxMotorKananTester->value()-10);    
}

void MainWindow::turnRight(){
    ui->spinBoxServoKananTester->setValue(ui->spinBoxServoKananTester->value()-10);
    ui->spinBoxServoKiriTester->setValue(ui->spinBoxServoKiriTester->value()-10);
}

void MainWindow::turnLeft(){
    ui->spinBoxServoKananTester->setValue(ui->spinBoxServoKananTester->value()+10);
    ui->spinBoxServoKiriTester->setValue(ui->spinBoxServoKiriTester->value()+10);
}

void MainWindow::connectDrone(){
    ui->pushButtonConnectDrone->setDisabled(true);
    ui->pushButtonDisconnectDrone->setEnabled(true);
    FILE *stream;
    stream = popen("ls /dev/ttyUSB*", "r");
    char b[30];
    while (!feof(stream)){
        if (fgets(b, 30, stream) != NULL){
            b[strlen(b)-1]=0;
            portNames.push_back(std::string(b));
        }
    }
    emit connectingDrone(&portNames);
}

void MainWindow::disconnectDrone(){
    ui->pushButtonConnectDrone->setEnabled(true);
    ui->pushButtonDisconnectDrone->setDisabled(true);
    emit disconnectingDrone();
}

void MainWindow::receiveDronePos(double lat, double lon){
    QPoint pt = calculatePoint(lat, lon);
    emit dronePosChanged(pt);
}

void MainWindow::droneRTL(){
    emit droneSetTraveling(2);
}

void MainWindow::droneLand(){
    emit droneSetTraveling(4);
}
