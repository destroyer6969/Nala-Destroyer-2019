#include "missionwrapper.h"
#include <QJsonArray>
#include <QJsonObject>

MissionWrapper::MissionWrapper(QObject *parent)
    :QObject(parent)
{
    normalServoMotor = 1450;
    normalServoKamera = 1500;
    portNum = -1;
    panicStop = 0;
    sysMode = 1;
//    instantiate all missions
    misi[MISI_NULL]         = NULL;
    misi[MISI_SLALOM]       = new Slalom();
    misi[MISI_AUTONAV]      = new AutoNav();
    misi[MISI_AUTODOCK]     = new AutoDocking();
    misi[MISI_FINDPATH]     = new FindThePath();
    misi[MISI_MHEADING]     = new MaintainHeading();
    misi[MISI_CIRCUMNAV]    = new CircumNav();
    misi[MISI_SPEEDCHAL]    = new SpeedChallenge();
    misi[MISI_RETURNHOME]   = new ReturnHome();
    misi[MISI_FOLLOWLEADER] = new FollowTheLeader();
    misi[MISI_RAISETHEFLAG] = new RaiseTheFlag();
    connect(misi[MISI_SLALOM], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_AUTONAV], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_AUTODOCK], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_FINDPATH], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_MHEADING], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_CIRCUMNAV], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_SPEEDCHAL], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_RETURNHOME], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_FOLLOWLEADER], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_RAISETHEFLAG], SIGNAL(arrived(int, const PositionType&)), this, SLOT(onArrived(int, const PositionType&)));
    connect(misi[MISI_SLALOM], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_AUTONAV], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_AUTODOCK], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_FINDPATH], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_MHEADING], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_CIRCUMNAV], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_SPEEDCHAL], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_RETURNHOME], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_FOLLOWLEADER], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_RAISETHEFLAG], SIGNAL(updateSudutJarak(double, double, double)), this, SLOT(getSudutJarak(double, double, double)));
    connect(misi[MISI_SLALOM], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_AUTONAV], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_AUTODOCK], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_FINDPATH], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_MHEADING], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_CIRCUMNAV], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_SPEEDCHAL], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_RETURNHOME], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_FOLLOWLEADER], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    connect(misi[MISI_RAISETHEFLAG], SIGNAL(sendOutput(int, int, int, int, int)), this, SLOT(sendToSTM(int, int, int, int, int)), Qt::DirectConnection);
    memset(outBuffer, 0, sizeof(outBuffer));
}

MissionWrapper::~MissionWrapper(){
    forceStop(1);
    for(int i=0; i<20; i++){
        sendToSTM(1500, 1500, 0, 0, 0);
        usleep(1000);
    }
    qDebug()<<"Mission Wrapper destroyed";
}

void MissionWrapper::setWaypoint(unsigned misiIndex, unsigned index, const PositionType &pos){
    if(misiIndex == 8){
        misi[MISI_RAISETHEFLAG]->setDroneWaypoint(index, pos);
    }
    else
        misi[misiIndex]->setWaypoint(index, pos);
}

const PositionType& MissionWrapper::getWaypoint(unsigned misiIndex, unsigned index){
    return misi[misiIndex]->getWaypoint(index);
}

void MissionWrapper::run(){
    forceStopped = 0;
    emit nextMission(misiIdx[1]);
    QMutex mtx1;
    mtx1.lock();
    it = 1;
    mtx1.unlock();
    while(it<7){
        emit sendControlLine(1, misi[misiIdx[it]]->getControlLine(0), misi[misiIdx[it]]->getControlLine(1), misi[misiIdx[it]]->getControlLine(2), misi[misiIdx[it]]->getControlLine(3));
        emit sendYThreshold(misi[misiIdx[it]]->getYThreshold());
        emit switchMissionTab(misiIdx[it]);
        emit displayCurrentMission(misiIdx[it]);
        // emit sendSRFMode(misi[misiIdx[it]]->srfMode);
        misi[misiIdx[it]]->startMission();
        emit clearOutputs();
        if(forceStopped)break;
        emit nextMission(misiIdx[it+1]);
        mtx1.lock();
        it++;
        mtx1.unlock();
    }
    if(panicStop){
        time_t tmp;
        time(&tmp);
        while((time(NULL)-tmp)<3){
            sendToSTM(1300, 1300, 0, 0, 0);
            usleep(1000);
        }
    }
    for(int i=0; i<20; i++){
        sendToSTM(1500, 1500, 0, 0, 0);
        usleep(1000);
    }
    panicStop = 0;
    emit endMission();
}

void MissionWrapper::onArrived(int nextDest, const PositionType& pos){
    emit nextDestChanged(nextDest, &pos);
}

void MissionWrapper::getSudutJarak(double sudut, double error, double jarak){
    emit sendSudutJarak(sudut, error, jarak);
}

void MissionWrapper::getCurrentPos(double latitude, double longitude, double compass){
    if (it>=7) return;
    mtx.lock();
    misi[misiIdx[it]]->currentPos.latitude = latitude;
    misi[misiIdx[it]]->currentPos.longitude = longitude;
    misi[misiIdx[it]]->compass = compass;
    mtx.unlock();
}

void MissionWrapper::getDetectedObject(const object_detection::ObjectAttr& msg){
    if(it>=7)return;
    misi[misiIdx[it]]->setObjAttr(msg);
}

void MissionWrapper::forceStop(bool){
    if (it>=7) return;
    if(misi[misiIdx[it]]->isRunning())misi[misiIdx[it]]->stop();
    forceStopped = 1;
    emit nextMission(misiIdx[1]);
}

void MissionWrapper::panic(){
    panicStop = 1;
    forceStop(1);
}

void MissionWrapper::changeMissionSequence(QString sequence){
    int tmp, idx=1;
    for(int i=0;i<21;i+=2){
        if(i>=sequence.length())break;
        tmp = sequence[i].digitValue();
        if(tmp>=0 && tmp<11){
            misiIdx[idx] = tmp;
            ++idx;
        }
    }
}

void MissionWrapper::getLidarObstacle(double xRange, double yRange){
    if (it>=7) return;
    misi[misiIdx[it]]->setLidarObstacle(xRange, yRange);
}

void MissionWrapper::write(QJsonObject &obj) const{
    QJsonArray misiArray;
    for(int i=1; i<NUM_MISI; i++){
        QJsonObject misiObject;
        misi[i]->write(misiObject);
        misiArray.append(misiObject);
    }
    obj["misi"] = misiArray;
}


void MissionWrapper::nextMission(){
    misi[misiIdx[it]]->stop();
}

void MissionWrapper::sendToSTM(int motorKanan, int motorKiri, int servoKanan, int servoKiri, int servoKamera){
    int outServo_ = normalServoMotor-servoKanan;
    int outSevoKamera = normalServoKamera+servoKamera;
    if(motorKanan>2000)motorKanan = 2000;if(motorKiri>2000)motorKiri = 2000;
    if(outServo_>2000)outServo_ = 2000;else if(outServo_<1000)outServo_ = 1000;
    mtx_.lock();
    sprintf(outBuffer, "A%.4d,%.4d,%.4d,%.4d,%.4dB", motorKanan, motorKiri, outServo_, outServo_, outSevoKamera);
    ::write(fd, outBuffer, strlen(outBuffer));
    mtx_.unlock();
    // qDebug()<<outBuffer;
}

void MissionWrapper::setControlLine(int topLeftX, int topLeftY, int botLeftX, int botLeftY, int topRightX, int topRightY, int botRightX, int botRightY){
    for(int i=1; i<10; i++)misi[i]->setControlLine(topLeftX, topLeftY, botLeftX, botLeftY, topRightX, topRightY, botRightX, botRightY);
}

int MissionWrapper::init_serial_port(const char *port_name){
	fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)return -1;
	else fcntl(fd, F_SETFL, 0);
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return 1;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 0;
	config.c_cc[VTIME] = 10;
	cfsetispeed(&config, B115200);
	cfsetospeed(&config, B115200);
	tcsetattr(fd, TCSAFLUSH, &config);
	return fd;
}

QPoint MissionWrapper::getCameraControlPoint(){
    if (it>=7) return QPoint(-6,-6);
    return misi[misiIdx[it]]->getCameraControlPoint();
}

void MissionWrapper::resetCameraControlPoint(){
    if (it>=7) return;
    misi[misiIdx[it]]->resetCameraControlPoint();
}

void MissionWrapper::runTest(){
    if(misi[misiIdx[it]]->isRunning())return;
    runningTest = 1;
    while(runningTest){
        sendToSTM(motorKananTest, motorKiriTest, servoKananTest, servoKiriTest, servoKameraTest);
        usleep(100);
    }
    for(int i=0; i<20; i++){
        sendToSTM(1500, 1500, 0, 0, 0);
        usleep(1000);
    }
}

void MissionWrapper::stopTest(){
    QMutex mtx1;
    mtx1.lock();
    runningTest = 0;
    mtx1.unlock();
}

void MissionWrapper::setMotorKananTest(int val){
    QMutex mtx1;
    mtx1.lock();
    motorKananTest = val;
    mtx1.unlock();
}

void MissionWrapper::setMotorKiriTest(int val){
    QMutex mtx1;
    mtx1.lock();
    motorKiriTest = val;
    mtx1.unlock();
}

void MissionWrapper::setServoKananTest(int val){
    QMutex mtx1;
    mtx1.lock();
    servoKananTest = val;
    mtx1.unlock();
}

void MissionWrapper::setServoKiriTest(int val){
    QMutex mtx1;
    mtx1.lock();
    servoKiriTest = val;
    mtx1.unlock();
}

void MissionWrapper::setServoKameraTest(int val){
    QMutex mtx1;
    mtx1.lock();
    servoKameraTest = val;
    mtx1.unlock();
}

void MissionWrapper::srfDataToMission(int srfL, int srfML, int srfMR, int srfR, int srfS){
    if (it>=7) return;
    misi[misiIdx[it]]->setSRF(srfL, srfML, srfMR, srfR, srfS);
    // qDebug()<qDebug()<<srfL<<", "<< srfML<<", "<< srfMR<<", "<<srfR<<", "<<srfS;
}

void MissionWrapper::lidarLeftObstacle(double range){
    if (it>=7) return;
    misi[misiIdx[it]]->setLidarLeftObstacle(range);
}

void MissionWrapper::connectToSTM(std::vector<std::string> *portNames){
    QMutex m;
    auto it=portNames->begin();
    int a=0;
    portNum = -1;
    char tmp[5];
    for(it; it!=portNames->end(); it++){
        if(init_serial_port("/dev/ttyUSB0") != -1){
            
            time(&startTime);
            while((time(NULL)-startTime)<1 && portNum==-1){
                result = read(fd, tmp, 5);
                qDebug()<<tmp;
                if(tmp[0]=='x'&&tmp[2]=='y'){
                    printf("Port \"%s\" connected to STM\n", std::string(*it).c_str());
                    m.lock();
                    opened = true;
                    portNum = a;
                    m.unlock();
                }
                usleep(1);
            }
            /*
            printf("Port \"%s\" connected to STM\n", std::string(*it).c_str());
            m.lock();
            opened = true;
            portNum = a;
            m.unlock();
            */
        }
        if(opened){
            break;
        }
        ++a;
    }
    if(!opened){
        printf("Error: cannot connect to STM\n");
        m.lock();
        portNum = -2;
        m.unlock();
    }
}

void MissionWrapper::readFromSTM(){
    if(opened){
        rMtx.lock();
        result = read(fd, inBuffer, 5);
        rMtx.unlock();
        if(inBuffer[0]=='x' && inBuffer[2]=='y'){
            if(inBuffer[1]-48==5 && sysMode!=2){
                sysMode=2;
                emit newSysMode(sysMode);
            }
            else if(inBuffer[1]-48==9 && sysMode!=1){
                sysMode=1;
                emit newSysMode(sysMode);
            }
            readCounter=0;
        }
        else{
            ++readCounter;
            if(readCounter>500){
                readCounter = 501;
                if(sysMode!=3) emit newSysMode(3);
                sysMode = 3;
            }
        }
    }
}

bool MissionWrapper::isOpened(){
    return opened;
}

int MissionWrapper::getPortNum(){
    return portNum;
}

char* MissionWrapper::getOutput(){
    return outBuffer;
}

char* MissionWrapper::getMissionControl(){
    return misi[misiIdx[it]]->getControl();
}

void MissionWrapper::setTengahServoMotor(int val){
    QMutex q;
    q.lock();
    normalServoMotor = val;
    q.unlock();
}

void MissionWrapper::setTengahServoKamera(int val){
    QMutex q;
    q.lock();
    normalServoKamera = val;
    q.unlock();
}
