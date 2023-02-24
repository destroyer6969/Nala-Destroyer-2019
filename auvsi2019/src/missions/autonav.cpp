#include "autonav.h"

AutoNav::AutoNav()
{
    init();
    clearWaypoints();
}

AutoNav::~AutoNav(){

}

void AutoNav::init(){
    NUM_WP = 0;
    speed       = 1500;
    this->num_pid = 2;
    outServo = servoKamera = 0;
    yThreshold = -6;
    topLeftX = 75; topLeftY = 340;
    botLeftX = 0; botLeftY = 410;
    mirrorControlLine();
    name = "Autonomous Navigation";
}

void AutoNav::setSudut(int sudut){
    this->sudut = sudut;
}

int AutoNav::startMission(){
    if(active && waypoints.size()){
        running = 1;
        wpIdx = 0;
        start();
        if(useCamera){
            object_detection::BoundingBox nearestRedCanBuoy, nearestGreenCanBuoy, nearestObstacle;
            qDebug()<<name<<" Started";
            xRangeLidar = yRangeLidar = -1;
            emit displayCurrentState("Go to the entrance");
            // Go to the entrance
            while(running && wpIdx<waypoints.size()-1){
                // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
                if(wpIdx<wpIdx<waypoints.size()-2)outServo = controlWaypoint(0);
                else outServo = controlWaypoint(0, 1, 0, 0);
                outSRFAvoidance = srfAvoidance();
                if(outSRFAvoidance!=-1){
                    if(outSRFAvoidance==0){
                        emit sendOutput(speedReverse, speedReverse, -50, -50, servoKamera);
                    }
                    else{
                        emit sendOutput(speed, speed, outSRFAvoidance, outSRFAvoidance, servoKamera);
                    }
                    usleep(100);
                    continue;
                }
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            emit displayCurrentState("Enter and exit Gate");        
            // Enter and exit gate
            time(&startTime);
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
                nearestRedCanBuoy = getNearestObject(1);
                nearestGreenCanBuoy = getNearestObject(2);
                // qDebug()<<nearestRedCanBuoy.xmax << ", " << nearestGreenCanBuoy.xmin;
                if(nearestRedCanBuoy.ymax > 0){
                    startPos.latitude = currentPos.latitude;
                    startPos.longitude = currentPos.longitude;
                    emit arrived(wpIdx, startPos);
                    if(nearestGreenCanBuoy.ymax > 0){
                        outServo = controlBetween2Object(1, nearestRedCanBuoy.xmax, nearestGreenCanBuoy.xmin);
                    }
                    else{
                        outServo = controlBoatSide(2, nearestRedCanBuoy.xmax, nearestRedCanBuoy.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                    }
                }
                else if(nearestGreenCanBuoy.ymax > 0){
                    startPos.latitude = currentPos.latitude;
                    startPos.longitude = currentPos.longitude;
                    emit arrived(wpIdx, startPos);
                    outServo = controlBoatSide(2, nearestGreenCanBuoy.xmin, nearestGreenCanBuoy.ymax, topRightX, topRightY, botRightX, botRightY);
                }
                else outServo = controlWaypoint(0);
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            resetCameraControlPoint();
        }
        else{
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
                // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
                outServo = controlWaypoint(0);
                outSRFAvoidance = srfAvoidance();
                if(outSRFAvoidance!=-1){
                    if(outSRFAvoidance==0){
                        emit sendOutput(speedReverse, speedReverse, -50, -50, servoKamera);
                    }
                    else{
                        outServo = outSRFAvoidance;
                        emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                    }
                    usleep(100);
                    continue;
                }
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
        }
        qDebug()<<name<<" Completed";
    }
    return nextMissionIndex;
}

void AutoNav::setNextDest(){
    wpIdx += 1;
}

void AutoNav::write(QJsonObject &obj) const{
    writeCommons(obj);

    // sudut
    obj["sudut"] = sudut;
}
