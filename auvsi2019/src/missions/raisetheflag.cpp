#include "raisetheflag.h"

RaiseTheFlag::RaiseTheFlag()
{
    init();
    clearWaypoints();
}

RaiseTheFlag::~RaiseTheFlag(){
    delete drone;
}

void RaiseTheFlag::init(){
    NUM_WP = 0;
    speed       = 1500;
    speed2      = 1500;
    sudut       = 0;
    dock        = 0;
    statusMission= "Waiting";
    name = "Raise The Flag";
    longitudeConst[REED_CANAL_PARK] = 9.771;
    longitudeConst[DANAU_8] = 11.03;
    latitudeConst = 11.12;
    location = DANAU_8;
    circleRadius = 350;
    radToDegrees = M_PI/(double)180.0;
    drone = new Drone();
    connect(this, SIGNAL(launchDrone()), drone, SLOT(launch()), Qt::QueuedConnection);
    connect(this, SIGNAL(stopDrone()), drone, SLOT(stop_traveling()), Qt::DirectConnection);
    droneThread = new QThread();
    drone->moveToThread(droneThread);
    droneThread->start();
}


void RaiseTheFlag::setSpeed2(int speed){
    this->speed2 = speed;
}


void RaiseTheFlag::setSudut(int sudut){
    this->sudut= sudut;
}

void RaiseTheFlag::setDroneWaypoint(unsigned index, const PositionType& pos){
    drone->setWaypoint(index, pos.latitude, pos.longitude);
}

void RaiseTheFlag::deleteDroneWaypoint(int index){
    mtx_pr.lock();
    drone->deleteWaypoint(index);
    mtx_pr.unlock();
}

void RaiseTheFlag::stop(){
    qDebug()<<"stop raise the flag";
    mtx_pr.lock();
    this->running = 0;
    drone->setTraveling(0);
    mtx_pr.unlock();
}

int RaiseTheFlag::startMission(){
    // -7,277583, 112,798236
    // running = 1;
    // while(running)drone->readMessage();
    // return 1;
    // dockLat = -7.277644;
    // dockLon = 112.798257;
    // this->compass = 180;
    // initDroneWaypoints();
    // emit launchDrone();
    // emit switchCam(1);
    // while(running && drone->getTravelingState()!=2)usleep(100);
    // sleep(10);
    // emit switchCam(0);
    if(active && waypoints.size()){
        object_detection::BoundingBox number, sevenSegment;
        object_detection::BoundingBox number1, number2, number3, number4;
        qDebug()<<name<<" Started";
        running = 1;
        wpIdx = 0;
        start();
        int outSpeed;
        // if(drone->isOpened())drone->toggle_offboard_control(1);
        emit displayCurrentState("Go to the entrance");
        // Go to the entrance
        while(running && wpIdx<waypoints.size()-5 && wpIdx<waypoints.size()){
            // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
            if(wpIdx<waypoints.size()-6){
                outSpeed = travelingSpeed;
                outServo = controlWaypoint(0);
            }
            else{
                outSpeed = speed;
                outServo = controlWaypoint(0, 1, 0, 0);
            }
            outSRFAvoidance = srfAvoidance();
            if(outSRFAvoidance!=-1){
                if(outSRFAvoidance==0){
                    emit sendOutput(speedReverse, speedReverse, -50, -50, servoKamera);
                }
                else{
                    emit sendOutput(outSpeed, outSpeed, outSRFAvoidance, outSRFAvoidance, servoKamera);
                }
                usleep(100);
                continue;
            }
            emit sendOutput(outSpeed, outSpeed, outServo, outServo, servoKamera);
            usleep(100);
            // qDebug()<<controlCircleWaypoint(circleRadius);
        }
        time(&startTime);
        lastWpIdx = wpIdx;
        int findDockWpIdx = wpIdx;
        emit displayCurrentState("Finding dock");
        bool launched = 0;
        // Finding dock
        while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
            if((wpIdx-findDockWpIdx)%2==0){
                outServo = controlWaypoint(0);
                number = getNearestObject(4);
                if(number.ymax<1) number = getNearestObject(5);
                if(number.ymax<1) number = getNearestObject(6);
                if(number.ymax<1) number = getNearestObject(7);
                if(number.ymax>0){
                    outServo = controlBetween2Object(2, number.xmax, number.xmin);
                }
                int qqqq=0;
                if(srfL<230)++qqqq;
                if(srfML<230)++qqqq;
                if(srfMR<230)++qqqq;
                if(srfR<230)++qqqq;
                if(qqqq>2){
                    getDockPosition(350);
                    int tmp = waypoints.size();
                    emit setNewWaypoint(dockLat, dockLon, waypoints.size());
                    while(running && waypoints.size()==tmp)usleep(100);
                    if(useSomething2){
                        emit displayCurrentState("Dock found! Turn right and launching drone");
                        if(useSomething && drone->isOpened()){
                            initDroneWaypoints();
                            emit launchDrone();
                            launched = 1;
                            qDebug()<<"Drone launched";
                        }
                        while(running && (srfL<240 || srfML<240 || srfR<240)){
                            emit sendOutput(speed, speed, 500, 500, servoKamera);
                            usleep(100);
                        }
                    }
                    else{
                        emit displayCurrentState("Dock found! Reverse and launching drone");
                        if(useSomething && drone->isOpened()){
                            initDroneWaypoints();
                            emit launchDrone();
                            launched = 1;
                            qDebug()<<"Drone launched";
                        }
                        controlCircleWaypoint(circleRadius);
                        while(running && this->jarak<circleRadius){
                            controlCircleWaypoint(circleRadius);
                            emit sendOutput(1400, 1400, 0, 0, servoKamera);
                            usleep(100);
                        }
                        usleep(500000);
                    }
                    setNextDest();
                    startPos.latitude = currentPos.latitude;
                    startPos.longitude = currentPos.longitude;
                    emit arrived(wpIdx, startPos);
                    break;
                }
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            else{
                outServo = controlWaypoint(0, 1, 0, 0);
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
        }
        for(int i=wpIdx;i<waypoints.size()-1;i++){
            setNextDest();
            startPos.latitude = currentPos.latitude;
            startPos.longitude = currentPos.longitude;
            emit arrived(wpIdx, startPos);
            usleep(1000);
        }
        if(!launched && drone->isOpened()){
            emit launchDrone();
            launched = 0;
            qDebug()<<"Drone launched";
        }
        int flagNumber;
        emit switchCam(1);
        time(&startTime);
        emit displayCurrentState("Finding seven segment");
        memset(sevenSegmentCounter, 0, sizeof(sevenSegmentCounter));
        while(running && (time(NULL)-startTime)<60 && drone->getTravelingState()!=2 && launched){
            sevenSegment = getNearestObject(8);
            if(sevenSegment.ymax>0)++sevenSegmentCounter[0];
            sevenSegment = getNearestObject(9);
            if(sevenSegment.ymax>0)++sevenSegmentCounter[1];
            sevenSegment = getNearestObject(10);
            if(sevenSegment.ymax>0)++sevenSegmentCounter[2];
            sevenSegment = getNearestObject(11);
            if(sevenSegment.ymax>0)++sevenSegmentCounter[3];
            if(useSomething2){
                controlWaypoint(0, 0);
                if(srfS>350)srfS = 15000;
                // qDebug()<<lidarControlRange-srfS;
                outServo = controlRange(1, lidarControlRange, srfS);
            }
            else{
                outServo = controlCircleWaypoint(circleRadius);
            }
            anotherSRFAvoidance();
            emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
            usleep(100);
        }
        emit switchCam(0);
        usleep(100000);
        int cntTmp = sevenSegmentCounter[0]>sevenSegmentCounter[1]? 0 : 1;
        int cntTmp0 = sevenSegmentCounter[2]>sevenSegmentCounter[3]? 2 : 3;
        flagNumber = sevenSegmentCounter[cntTmp]>sevenSegmentCounter[cntTmp0]? cntTmp : cntTmp0;
        // /* <Sementara */ flagNumber=1; /* Sementara>*/
        int flagCounter=0;
        char dispStr[30];
        sprintf(dispStr, "Finding flag %d", flagNumber+1);
        emit displayCurrentState(dispStr);
        qDebug()<<flagNumber+1;
        emit sendFlag(flagNumber+1);
        // Finding flag
        while(running && (time(NULL)-startTime)<150){
            if(useSomething2){
                controlWaypoint(0, 0);
                if(srfS>350)srfS = 15000;
                // qDebug()<<lidarControlRange-srfS;
                outServo = controlRange(1, lidarControlRange, srfS);
            }
            else{
                outServo = controlCircleWaypoint(circleRadius);
            }
            number1 = getNearestObject(4);
            number2 = getNearestObject(5);
            number3 = getNearestObject(6);
            number4 = getNearestObject(7);
            number = getNearestObject(flagNumber+4);
            if(number.ymax>0 && number.ymax>=number1.ymax && number.ymax>=number2.ymax
                     && number.ymax>=number3.ymax && number.ymax>=number4.ymax){
                int numberX = (number.xmax+number.xmin)/2;
                if(numberX>0 && numberX<370 && newImage){
                    newImage = 0;
                    ++flagCounter;
                    if(flagCounter>6){
                        time(&startTime);
                        emit displayCurrentState("Waiting 4 secs");
                        while(running && (time(NULL)-startTime)<4){
                            outServo = controlCircleWaypoint(circleRadius);
                            emit sendOutput(speedSlow, speedSlow, outServo, outServo, -830);
                            usleep(100);
                        }
                        break;
                    }
                }
                else if(newImage) flagCounter=0;
            }
            anotherSRFAvoidance();
            emit sendOutput(speedSlow, speedSlow, outServo, outServo, -830);
            usleep(100);
        }
        lastWpIdx = wpIdx;
        emit displayCurrentState("Chasing flag");
        time(&startTime);
        // Chasing flag
        while(running && (time(NULL)-startTime)<15){
            int qqqq = 0;
//            if(launched){
//
//            }
//            else{
//                number = getNearestObject(5);
//                if(number.ymax<0) number = getNearestObject(6);
//                if(number.ymax<0) number = getNearestObject(7);
//                if(number.ymax<0) number = getNearestObject(8);
//            }
            number1 = getNearestObject(4);
            number2 = getNearestObject(5);
            number3 = getNearestObject(6);
            number4 = getNearestObject(7);
            number = getNearestObject(flagNumber+4);
            outServo = controlWaypoint(0, 1, 0, 0);
            if(this->errorSudut<-15){
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, -830);
            }
            else{
                if(number.ymax>0 && number.ymax>=number1.ymax && number.ymax>=number2.ymax
                     && number.ymax>=number3.ymax && number.ymax>=number4.ymax)
                        outServo = controlBetween2Object(2, number.xmax, number.xmin);
                if(srfL<35)++qqqq;
                if(srfML<35)++qqqq;
                if(srfMR<35)++qqqq;
                if(srfR<35)++qqqq;
                if(qqqq>2){
                    setNextDest();
                    startPos.latitude = currentPos.latitude;
                    startPos.longitude = currentPos.longitude;
                    emit arrived(wpIdx, startPos);
                    break;
                }
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
            }
            usleep(100);
        }
        emit displayCurrentState("Pushing button");
        time(&startTime);
        // Pushing button
        while(running && (time(NULL)-startTime)<2){
            emit sendOutput(speed, speed, 0, 0, servoKamera);
            usleep(100);            
        }
        emit displayCurrentState("Reverse");
        time(&startTime);
        // Reverse
        while(running && (time(NULL)-startTime)<4){
            emit sendOutput(1400, 1400, 0, 0, servoKamera);            
            usleep(100);            
        }
        qDebug()<<name<<" Finished";
    }
    return nextMissionIndex;
}

void RaiseTheFlag::setNextDest(){
    ++wpIdx;
}

void RaiseTheFlag::write(QJsonObject &obj) const{
    writeCommons(obj);

    obj["speed2"] = speed2;
    obj["sudut"] = sudut;
}

void RaiseTheFlag::getDockPosition(double range){
    dockLat = this->currentPos.latitude + (cos(this->compass*radToDegrees) * range / latitudeConst * 0.000001);
    dockLon = this->currentPos.longitude + (sin(this->compass*radToDegrees) * range / longitudeConst[location] * 0.000001);
}

int RaiseTheFlag::controlCircleWaypoint(double radius, bool direction){
    // direction 0 == counter clockwise, direction 1 == clockwise, default = 1
    mtx10.lock();
    strcpy(control, "Circle waypoint");
    mtx10.unlock();
    errorSebelum = error;
    sudutTujuan = (atan2(dockLat - currentPos.latitude,
                        currentPos.longitude - dockLon)
                    / radToDegrees) - 90;
    sudutTujuan = std::fmod((sudutTujuan + 360.0), 360.0);
    jarak       = sqrtf64(
                pow(dockLat - currentPos.latitude, 2)
                + pow(currentPos.longitude - dockLon, 2)
                ) * 11000000.0;
    errorDistSebelum = errorDist;
    if(direction){
        errorDist = radius - jarak;
        errorSudut  = std::fmod((sudutTujuan - compass + 360.0), 360.0) + 90;
    }
    else{
        errorDist = jarak - radius;
        errorSudut  = std::fmod((sudutTujuan - compass + 360.0), 360.0) - 90;
    }
    if(errorSudut>180)errorSudut -= 360;
    if(errorSudut<45 || errorSudut>-45){
        controlDistanceOut = (int)(pid[0].p*errorSudut + pid[0].i*(errorSudut+errorSebelum) + pid[0].d*(errorSudut-errorSebelum)) + (int)(pid[3].p * errorDist + pid[3].d * (errorDist - errorDistSebelum) + pid[3].i * (errorDist + errorDistSebelum));
    }
    else controlDistanceOut = (int)(pid[0].p*errorSudut + pid[0].i*(errorSudut+errorSebelum) + pid[0].d*(errorSudut-errorSebelum));
    return controlDistanceOut;
}

void RaiseTheFlag::droneCamOn(){
    emit switchCam(1);
}

void RaiseTheFlag::droneCamOff(){
    emit switchCam(0);
}

void RaiseTheFlag::anotherSRFAvoidance(){
    if(srfL<230 || srfML<230 || srfMR<230 || srfR<230){
        outServo = 500;
        mtx10.lock();
        strcpy(control, "SRF turn right");
        mtx10.unlock();
    }
    else if(srfS<200){
        outServo = controlRange(1, lidarControlRange, srfS);
        mtx10.lock();
        strcpy(control, "Lidar control");
        mtx10.unlock();
    }
}

void RaiseTheFlag::initDroneWaypoints(){
    PositionType p, dock;
    dock.latitude = dockLat;
    dock.longitude = dockLon;
    double range = 300;
    p.latitude = dockLat + (cos(this->compass*radToDegrees) * range / latitudeConst * 0.000001);
    p.longitude = dockLon + (sin(this->compass*radToDegrees) * range / longitudeConst[location] * 0.000001);
    setDroneWaypoint(0, p);
    p.latitude = dockLat + (cos((this->compass+(double)180)*radToDegrees) * range / latitudeConst * 0.000001);
    p.longitude = dockLon + (sin((this->compass+(double)180)*radToDegrees) * range / longitudeConst[location] * 0.000001);
    setDroneWaypoint(1, p);
    p.latitude = dockLat + (cos((this->compass+(double)90)*radToDegrees) * range / latitudeConst * 0.000001);
    p.longitude = dockLon + (sin((this->compass+(double)90)*radToDegrees) * range / longitudeConst[location] * 0.000001);
    setDroneWaypoint(2, p);
    p.latitude = dockLat + (cos((this->compass+(double)270)*radToDegrees) * range / latitudeConst * 0.000001);
    p.longitude = dockLon + (sin((this->compass+(double)270)*radToDegrees) * range / longitudeConst[location] * 0.000001);
    setDroneWaypoint(3, p);
    setDroneWaypoint(4, dock);
}
