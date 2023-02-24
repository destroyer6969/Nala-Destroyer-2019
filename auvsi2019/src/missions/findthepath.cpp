#include "findthepath.h"

FindThePath::FindThePath()
{
    init();
    clearWaypoints();
}

FindThePath::~FindThePath(){

}

void FindThePath::init(){
    NUM_WP = 0;
    speed       = 1500;
    mode        = 0;
    name = "Find The Path";
    topLeftX = 80; topLeftY = 410;
    botLeftX = 30; botLeftY = 480;
    yThreshold = 420;
    srfThresholdJauh = 45;
    srfThresholdDeket = 35;
    mirrorControlLine();
}

void FindThePath::setMode(int mode){
    this->mode = mode;
}


int FindThePath::startMission(){
    if(active && waypoints.size()){
        object_detection::BoundingBox nearestObstacle, nearestAnotherCanBuoy, canBuoyTemp, nearestBall;
        running = 1;
        wpIdx = 0;
        start();
        qDebug()<<name<<" Started";
        int outSpeed;
        int xThreshold=320;
        if(useCamera){
            if(waypoints.size()>1)emit setNewWaypoint(waypoints[waypoints.size()-2].latitude, waypoints[waypoints.size()-2].longitude, waypoints.size());
            emit displayCurrentState("Go to the entrance");
            // Go to the entrance
            while(running && wpIdx<waypoints.size()-2){
                // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
                if(wpIdx<waypoints.size()-3){
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
                        emit sendOutput(speed, speed, outSRFAvoidance, outSRFAvoidance, servoKamera);
                    }
                    usleep(100);
                    continue;
                }
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            lastWpIdx = wpIdx;
            time(&startTime);
            emit displayCurrentState("Finding the path in");
            // Finding the path in
            while(running && (time(NULL)-startTime)<timeout && wpIdx==lastWpIdx && wpIdx<waypoints.size()){
                nearestObstacle = getNearestObstacle(0, yThreshold);
                nearestAnotherCanBuoy = getNearestObject(3);
                if(nearestAnotherCanBuoy.ymax>0)canBuoyTemp = nearestAnotherCanBuoy;
                /*if(xRangeLidar<-1 && yRangeLidar<50){
                    speedSlow = 1425;
                    outServo = controlLidarAvoidance(3, 150);
                }*/
                if(waypoints.size()>3){
                    outServo = controlWaypoint(0, 0);
                }
                if(nearestObstacle.ymax > 0){
                    if(nearestAnotherCanBuoy.ymax>nearestObstacle.ymax){
                        outServo = controlBoatSide(1, canBuoyTemp.xmax, canBuoyTemp.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                    }
                    else{
                        if(nearestAnotherCanBuoy.ymax<1 && canBuoyTemp.ymax>0){
                            if((canBuoyTemp.xmin+canBuoyTemp.xmax)/2 < 320) xThreshold = 200;
                            else xThreshold = 440;
                        }
                        else xThreshold = 320;
                        if((nearestObstacle.xmin+nearestObstacle.xmax)/2 < xThreshold){
                            outServo = controlBoatSide(2, nearestObstacle.xmax, nearestObstacle.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                        }
                        else{
                            outServo = controlBoatSide(2, nearestObstacle.xmin, nearestObstacle.ymax, topRightX, topRightY, botRightX, botRightY);
                        }
                    }
                }
                else if(canBuoyTemp.ymax>0){
                    if(nearestAnotherCanBuoy.ymax>0) outServo = controlBoatSide(3, canBuoyTemp.xmax, canBuoyTemp.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                    else outServo = controlBetween2Object(1, canBuoyTemp.xmax, canBuoyTemp.xmin);
                }
                else{
                    mtx10.lock();
                    strcpy(control, "Waypoint");
                    mtx10.unlock();
                }
                if(nearestAnotherCanBuoy.xmax<200 && nearestAnotherCanBuoy.ymax>yThreshold){
                    setNextDest();
                    startPos.latitude = currentPos.latitude;
                    startPos.longitude = currentPos.longitude;
                    emit arrived(wpIdx, waypoints[wpIdx]);
                    emit displayCurrentState("Passing Can Buoy");
                    while(running && nearestAnotherCanBuoy.ymax>0){
                        nearestAnotherCanBuoy = getNearestObject(3);
                        emit sendOutput(speedSlow, speedSlow, 500, 500, servoKamera);
                        usleep(100);
                    }
                    time_t tmpTime;
                    time(&tmpTime);
                    while(running && (time(NULL)-tmpTime)<3){
                        nearestAnotherCanBuoy = getNearestObject(3);
                        if(nearestAnotherCanBuoy.ymax>0)emit sendOutput(speedSlow, speedSlow, 500, 500, servoKamera);
                        else emit sendOutput(speedSlow, speedSlow, 0, 0, servoKamera);
                        usleep(100);
                    }
                    break;
                }
                if(anotherSRFAvoidance()){
                    emit sendOutput(1425, 1425, outServo, outServo, servoKamera);
                    usleep(100);
                    continue;
                }
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
                usleep(100);
            }
            startPos.latitude = currentPos.latitude;
            startPos.longitude = currentPos.longitude;
            emit arrived(wpIdx, startPos);
            controlWaypoint(0, 1, 1);
            int pidTmp = 0;
            emit displayCurrentState("Circling around");
            // Circling around
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout && (errorSudut<-35 || errorSudut>15)){
                // if(errorSudut<-90)pidTmp = 3;
                // if(pidTmp==3 && errorSudut>-90)pidTmp = 0;
                outServo = controlWaypoint(pidTmp, 1, 1);
                if(outServo<servoMuterMax)outServo = servoMuterMax;
                nearestObstacle = getNearestObstacle(0, yThreshold);
                nearestAnotherCanBuoy = getNearestObject(3);
                nearestBall = getNearestObject(0);

                // qDebug()<<"++++++"<<outServo;
                
                /*if(xRangeLidar<-1 && yRangeLidar<50){
                    speedSlow = 1425;
                    outServo = controlLidarAvoidance(3, 150);
                }*/
                if(nearestAnotherCanBuoy.ymax>0){
                    //outServo = 500;//controlBoatSide(1, nearestAnotherCanBuoy.xmax, nearestAnotherCanBuoy.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                    emit displayCurrentState("Passing Can Buoy");
                    while(running && nearestAnotherCanBuoy.ymax>0){
                        nearestAnotherCanBuoy = getNearestObject(3);
                        emit sendOutput(speedSlow, speedSlow, 500, 500, servoKamera);
                        usleep(100);
                    }
                    time_t tmpTime;
                    time(&tmpTime);
                    while(running && (time(NULL)-tmpTime)<3){
                        nearestAnotherCanBuoy = getNearestObject(3);
                        if(anotherSRFAvoidance()){
                            emit sendOutput(1425, 1425, outServo, outServo, servoKamera);
                            usleep(100);
                            continue;
                        }
                        if(nearestAnotherCanBuoy.ymax>0)emit sendOutput(speedSlow, speedSlow, 500, 500, servoKamera);
                        else emit sendOutput(speedSlow, speedSlow, 0, 0, servoKamera);
                        usleep(100);
                    }
                    emit displayCurrentState("Circling around");
                }
                // else if(nearestObstacle.xmax > 0){
                //     if((nearestObstacle.xmin+nearestObstacle.xmax)/2 < 160){
                //         outServo = controlBoatSide(2, nearestObstacle.xmax, nearestObstacle.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                //     }
                //     else{
                //         outServo = controlBoatSide(2, nearestObstacle.xmin, nearestObstacle.ymax, topRightX, topRightY, botRightX, botRightY);
                //     }
                // }
                // qDebug()<<"======="<<outServo;
                if(anotherSRFAvoidance()){
                    emit sendOutput(1425, 1425, outServo, outServo, servoKamera);
                    usleep(100);
                    continue;
                }
                // qDebug()<<"~~~~~~"<<outServo;
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
                usleep(100);
            }
            emit displayCurrentState("Finding the path out");
            // Find the path out
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
                nearestObstacle = getNearestObstacle(0, yThreshold);
                nearestAnotherCanBuoy = getNearestObject(3);
                outServo = controlWaypoint(0, 0);
                if(nearestObstacle.ymax > 0){
                    if(nearestAnotherCanBuoy.ymax>nearestObstacle.ymax){
                        outServo = controlBoatSide(1, canBuoyTemp.xmax, canBuoyTemp.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                    }
                    else{
                        if(errorDist > 0) xThreshold = 200;
                        else xThreshold = 440;
                        if((nearestObstacle.xmin+nearestObstacle.xmax)/2 < xThreshold){
                            outServo = controlBoatSide(2, nearestObstacle.xmax, nearestObstacle.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                        }
                        else{
                            outServo = controlBoatSide(2, nearestObstacle.xmin, nearestObstacle.ymax, topRightX, topRightY, botRightX, botRightY);
                        }
                    }
                }
                else if(nearestAnotherCanBuoy.ymax>0){
                    outServo = controlBoatSide(2, nearestAnotherCanBuoy.xmax, nearestAnotherCanBuoy.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                }
                else{
                    mtx10.lock();
                    strcpy(control, "Waypoint");
                    mtx10.unlock();
                }
                if(anotherSRFAvoidance()){
                    emit sendOutput(1425, 1425, outServo, outServo, servoKamera);
                    usleep(100);
                    continue;
                }
                emit sendOutput(speedSlow, speedSlow, outServo, outServo, servoKamera);
                usleep(100);
            }
        }
        else{
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
                // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
                if(wpIdx==0)outSpeed = travelingSpeed;
                else outSpeed = speed;
                outServo = controlWaypoint(0);
                outSRFAvoidance = srfAvoidance();
                if(outSRFAvoidance!=-1){
                    if(outSRFAvoidance==0){
                        emit sendOutput(1425, 1425, -50, -50, servoKamera);
                    }
                    else{
                        outServo = outSRFAvoidance;
                        emit sendOutput(outSpeed, outSpeed, outServo, outServo, servoKamera);
                    }
                    usleep(100);
                    continue;
                }
                emit sendOutput(outSpeed, outSpeed, outServo, outServo, servoKamera);
                usleep(100);
            }
        }
        emit signalDeleteWaypoint(waypoints.size()-1);
        resetCameraControlPoint();
        qDebug()<<name<<" Finished";
    }
    return nextMissionIndex;
}

void FindThePath::setNextDest(){
    ++wpIdx;
}

void FindThePath::write(QJsonObject &obj) const{
    writeCommons(obj);
}

bool FindThePath::equalBox(object_detection::BoundingBox &box1, object_detection::BoundingBox &box2){
    if(box1.xmin!=box2.xmin)return 0;
    if(box1.xmax!=box2.xmax)return 0;
    if(box1.ymin!=box2.ymin)return 0;
    if(box1.ymax!=box2.ymax)return 0;
    return 1;
}

bool FindThePath::anotherSRFAvoidance(){
    if(srfL<srfThresholdDeket || srfML<srfThresholdDeket || srfMR<srfThresholdDeket || srfR<srfThresholdDeket){
        outServo = 0;
        mtx10.lock();
        strcpy(control, "SRF reverse");
        mtx10.unlock();
    }
    else if(srfL<srfThresholdJauh || srfML<srfThresholdJauh){
        if(srfMR>srfThresholdJauh && srfR>srfThresholdJauh){
            outServo = 500;
            mtx10.lock();
            strcpy(control, "SRF turn right");
            mtx10.unlock();
        }
        else{
            outServo = 0;
            mtx10.lock();
            strcpy(control, "SRF reverse");
            mtx10.unlock();
        }
    }
    else if(srfMR<srfThresholdJauh || srfR<srfThresholdJauh){
        outServo = -500;
        mtx10.lock();
        strcpy(control, "SRF turn left");
        mtx10.unlock();
    }
    if(outServo==0)return 1;
    return 0;
}
