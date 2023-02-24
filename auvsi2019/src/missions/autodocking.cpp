#include "autodocking.h"
#include "QDebug"

AutoDocking::AutoDocking()
{
    init();
    clearWaypoints();
}

AutoDocking::~AutoDocking(){

}

void AutoDocking::init(){
    NUM_WP = 0;
    threshold   = 0;
    sudut       = 0;
    dock        = 0;
    maxPinger   = 0;
    frequency   = 1000;
    counter     = 0;
    pinger      = 0;
    speed2      = 1650;
    yThreshold  = -6;
    statusMission= "Waiting";
    name = "Automated Docking";
    srfMode = "00";
}

void AutoDocking::setSpeed2(int speed){
    mtx3.lock();
    this->speed2  = speed;
    mtx3.unlock();
}


void AutoDocking::setThreshold(int threshold){
    mtx3.lock();
    this->threshold = threshold;
    mtx3.unlock();
}


void AutoDocking::setSudut(int sudut){
    mtx3.lock();
    this->sudut = sudut;
    mtx3.unlock();
}


void AutoDocking::setMaxPinger(int maxPinger){
    mtx3.lock();
    this->maxPinger= maxPinger;
    mtx3.unlock();
}

void AutoDocking::setFrequency(int frequency){
    mtx3.lock();
    this->frequency = frequency;
    mtx3.unlock();
}

int AutoDocking::startMission(){
    if(active && waypoints.size()){
        qDebug()<<name<<" Started";
        running = 1;
        wpIdx = 0;
        start();
        int outSpeed, numberX1, numberX2, numberX3, numberXTmp, numberX;
        object_detection::BoundingBox number1, number2, number3;
        emit displayCurrentState("Go to the entrance");
        // Go to the entrancebreak;
        while(running && wpIdx<waypoints.size()-7 && wpIdx<waypoints.size()){
            // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
            if(wpIdx<waypoints.size()-8 && wpIdx!=0){
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
        }
        time(&startTime);
        if(useSomething){
            lastWpIdx = wpIdx;
            emit displayCurrentState("Finding dock");
            int qqqq=0;
            int QQQQ=0;
            // Finding dock
            while(running && wpIdx!=breakWPIndex && (time(NULL)-startTime)<timeout && lastWpIdx==wpIdx){
                outServo = controlWaypoint(0);
                qqqq=0;
                if(srfL<230)++qqqq;
                // else --qqqq;
                if(srfML<230)++qqqq;
                // else --qqqq;
                if(srfMR<230)++qqqq;
                // else --qqqq;
                if(srfR<230)++qqqq;
                // else --qqqq;
                if(qqqq>2){
                    emit displayCurrentState("Dock found!");
                    while(running && (srfL<240 || srfML<240|| srfMR<240 || srfR<240)){
                        /*
                        if(srfL>150)++QQQQ;
                        else --QQQQ;
                        if(srfML>150)++QQQQ;
                        else --QQQQ;
                        if(srfR>150)++QQQQ;
                        else --QQQQ;
                        */
                        emit sendOutput(speed, speed, 500, 500, servoKamera);
                        usleep(100);
                    }
                    setNextDest();
                    startPos.latitude = currentPos.latitude;
                    startPos.longitude = currentPos.longitude;
                    emit arrived(wpIdx, startPos);
                    break;
                }
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
                usleep(100);
            }
            int tmp = waypoints.size();
            emit setNewWaypoint(waypoints[waypoints.size()-2].latitude, waypoints[waypoints.size()-2].longitude, waypoints.size());
            while(running && waypoints.size()==tmp)usleep(100);
            double range = 0, amplitudo = -1, maxAmplitudo = -1;
            controlWaypoint(0, 0);
            srand(time(0));
            emit displayCurrentState("Finding beacon");
            PositionType t;
            t.latitude = currentPos.latitude;
            t.longitude = currentPos.longitude;
            // Finding beacon
            while(running && (time(NULL)-startTime)<timeout && (errorSudut>15 || errorSudut<-15)){
                controlWaypoint(0, 0);
                amplitudo = (rand() % (51 - 50 + 1)) + 50; //read from hydrophone
                if(maxAmplitudo<amplitudo){
                    maxAmplitudo = amplitudo;
                    t.latitude = currentPos.latitude;
                    t.longitude = currentPos.longitude;
                }
                if(srfS>350)srfS = 1500;
                outServo = controlRange(1, lidarControlRange, srfS);
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
                usleep(100);
            }
            tmp = waypoints.size();
            emit setNewWaypoint(t.latitude, t.longitude, waypoints.size());
            while(running && waypoints.size()==tmp)usleep(100);
            startPos.latitude = currentPos.latitude;
            startPos.longitude = currentPos.longitude;
            emit arrived(wpIdx, startPos);
            emit displayCurrentState("Get ready for docking");
            // Get ready for docking
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
                // int outTmp = controlRange(1, lidarControlRange, srfS);
                outServo = controlWaypoint(0);
                /*if(outTmp>0){
                    outServo+=outTmp;
                    qDebug()<<outTmp;
                }*/
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
        }
        lastWpIdx = wpIdx;
        int findDockWpIdx = wpIdx;
        int dockNumber = 2;
        emit displayCurrentState("Go to dock");
        // Go to dock
        while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
            outServo = controlWaypoint(0);
            if((wpIdx-findDockWpIdx)%3==0){
                /*
                numberX = 0;
                number1 = getNearestObject(4);
                numberX1 = abs(((number1.xmax+number1.xmin)/2)-320);
                number2 = getNearestObject(5);
                numberX2 = abs(((number2.xmax+number2.xmin)/2)-320);
                number3 = getNearestObject(6);
                numberX3 = abs(((number3.xmax+number3.xmin)/2)-320);
                if(numberX1>numberX2){
                    numberXTmp = (number1.xmax+number1.xmin)/2;
                    lastDock = 1;
                }
                else{
                    numberXTmp = (number2.xmax-number2.xmin)/2;
                    lastDock = 2;
                }
                if(numberXTmp>numberX3){
                    numberX = numberXTmp;
                }
                else{
                    numberX = (number3.xmax+number3.xmin)/2;
                    lastDock = 3;
                }
                */
                number1 = getNearestObject(dockNumber+3);
                if(number1.ymax>0){
                    outServo = controlBetween2Object(2, number1.xmax, number1.xmin);
                }
            }
            // if(yRangeLidar!=-1 && yRangeLidar<20)break;
            int qqqq=0;
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
            emit sendOutput(speed, speed, outServo, outServo, servoKamera);
            usleep(100);
        }
        emit sendDock(dockNumber);
        emit displayCurrentState("Hold Position");
        time(&startTime);
        // Hold position
        while(running && (time(NULL)-startTime)<6){
            emit sendOutput(speed, speed, 0, 0, servoKamera);
            usleep(100);            
        }
        emit displayCurrentState("Reverse");
        time(&startTime);
        // Reverse
        while(running && (time(NULL)-startTime)<6){
            emit sendOutput(1400, 1400, 0, 0, servoKamera);            
            usleep(100);            
        }
        qDebug()<<name<<" Finished";
    }
    return nextMissionIndex;
}

void AutoDocking::setNextDest(){
    ++wpIdx;
}

void AutoDocking::write(QJsonObject &obj) const{
    writeCommons(obj);

    obj["speed2"]       = speed2;
    obj["threshold"]    = threshold;
    obj["sudut"]        = sudut;
    obj["maxPinger"]    = maxPinger;
    obj["frequency"]    = frequency;
}
