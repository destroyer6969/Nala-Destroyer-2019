#include "speedchallenge.h"

SpeedChallenge::SpeedChallenge()
{
    init();
    clearWaypoints();
}

SpeedChallenge::~SpeedChallenge(){

}


void SpeedChallenge::init(){
    NUM_WP = 0;
    speed       = 1500;
    name  = "Speed Challange";
    topLeftX = 120; topLeftY = 420;
    botLeftX = 50; botLeftY = 480;
    yThreshold = 0;
    servoMuterMax = 0;
}

int SpeedChallenge::startMission(){
    if(active && waypoints.size()){
        running = 1;
        wpIdx = 0;
        lastWpIdx = wpIdx;
        pidTmp.p = pidDist.p; pidTmp.i = pidDist.i; pidTmp.d = pidDist.d;
        int outSpeed;
        start();
        if(useCamera){
            int m=2;
            object_detection::BoundingBox nearestRedBall, nearestGreenBall, nearestBlueBall, nearestObstacle;
            qDebug()<<name<<" Started";
            int tmp = waypoints.size();
            emit setNewWaypoint(currentPos.latitude, currentPos.longitude, waypoints.size());
            emit setNewWaypoint(waypoints[waypoints.size()-2].latitude, waypoints[waypoints.size()-2].longitude, waypoints.size());
            while(running && waypoints.size()==tmp)usleep(100);
            tmp = waypoints.size();
            emit setNewWaypoint(waypoints[waypoints.size()-3].latitude, waypoints[waypoints.size()-3].longitude, waypoints.size());
            while(running && waypoints.size()==tmp)usleep(100);

            object_detection::BoundingBox ballTemp;
            int ballLostCounter = 0;
            emit displayCurrentState("Go to the entrance");
            // Go to the entrance
            while(running && wpIdx<waypoints.size()-3){
                qDebug()<<wpIdx;
                // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
                if(wpIdx<waypoints.size()-4){
                    outSpeed = travelingSpeed;
                    outServo = controlWaypoint(0);
                }
                else{
                    pidDist.p = pid[3].p; pidDist.i = pid[3].i; pidDist.d = pid[3].d;
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
            mtx5.lock();
            controlWaypoint(0, 0);
            mtx5.unlock();
            double jarakTemp = this->jarak/2.0;
            time(&startTime);
            if(0){
                emit displayCurrentState("Entering gate");
                // Entering gate
                while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout && this->jarak>jarakTemp){
                    nearestRedBall = getNearestObject(0, 0);
                    nearestGreenBall = getNearestObject(0,1);
                    mtx5.lock();
                    outServo = controlWaypoint(0, 0);
                    if(nearestRedBall.ymax > 0){
                        if(nearestGreenBall.ymax > 0 && nearestRedBall.xmin < nearestGreenBall.xmin){
                            outServo = controlBetween2Object(1, nearestRedBall.xmax, nearestGreenBall.xmin);
                        }
                        else{
                            int tmp = (m+1)*nearestRedBall.xmax-m*nearestRedBall.xmin;
                            outServo = controlBetween2Object(1, tmp, tmp);
                        }
                    }
                    else if(nearestGreenBall.ymax > 0){
                        int tmp = (m+1)*nearestGreenBall.xmin-m*nearestGreenBall.xmax;
                        outServo = controlBetween2Object(1, tmp, tmp);
                    }
                    else{
                        mtx10.lock();
                        strcpy(control, "Waypoint");
                        mtx10.unlock();
                    }
                    mtx5.unlock();
                    emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                    if(nearestRedBall.ymax>yThreshold || nearestGreenBall.ymax>yThreshold)break;
                    usleep(100);
                }
            }

            lastWpIdx = wpIdx;
            emit displayCurrentState("Go to blue ball");
            // Go to blue ball
            while(running && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout && this->jarak>jarakTemp){
                /*if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
                else */outServo = controlWaypoint(0);
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            emit displayCurrentState("Seeking blue ball");
            // Seeking blue ball
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout && wpIdx==lastWpIdx){
                nearestBlueBall = getNearestObject(0, 2);
                mtx5.lock();
                outServo = controlWaypoint(0);
                if(nearestBlueBall.ymax > 0){
                    int tmp = (m+1)*nearestBlueBall.xmax-m*nearestBlueBall.xmin;
                    outServo = controlBetween2Object(1, tmp, tmp);
                    // outServo = controlBoatSide(1, nearestBlueBall.xmax, nearestBlueBall.ymax, topLeftX, topLeftY, botLeftX, botLeftY);
                }
                mtx5.unlock();
                if(nearestBlueBall.xmax<120 && nearestBlueBall.ymax>yThreshold){
                    setNextDest();
                    emit arrived(wpIdx, waypoints[wpIdx]);
                    break;
                }
                else{
                    mtx10.lock();
                    strcpy(control, "Waypoint");
                    mtx10.unlock();
                }
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            startPos.latitude = currentPos.latitude;
            startPos.longitude = currentPos.longitude;
            emit arrived(wpIdx, startPos);
            emit displayCurrentState("Passing blue ball");
            time_t tmpTime;
            time(&tmpTime);
            while(running && (time(NULL)-tmpTime)<1){
                emit sendOutput(speed, speed, 0, 0, servoKamera);
                usleep(100);
            }
            lastWpIdx = wpIdx;
            int pidTmp=0;
            controlWaypoint(pidTmp, 0, 1);
            emit displayCurrentState("Circling around blue ball");
            // Circling around blue ball
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout && errorSudut<-15){
                nearestBlueBall = getNearestObject(0, 2);
                mtx5.lock();
                outServo = controlWaypoint(0, 1, 1, 0);
                if(outServo<servoMuterMax)outServo = servoMuterMax;
                /*if(xRangeLidar!=-1) outServo = controlLidarAvoidance(1, 150);
                else */if(nearestBlueBall.ymax > yThreshold){
                    int tmp = (m+1)*nearestBlueBall.xmax-m*nearestBlueBall.xmin;
                    outServo = controlBetween2Object(1, tmp, tmp);
                }
                mtx5.unlock();
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            emit displayCurrentState("Exiting gate");
            controlWaypoint(0, 0);
            jarakTemp = this->jarak/1.5;
            // Exit gate
            while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size() && (time(NULL)-startTime)<timeout){
                // nearestRedBall = getNearestObject(0, 0);
                // nearestGreenBall = getNearestObject(0,1);
                mtx5.lock();
                if(this->jarak>jarakTemp)outServo = controlWaypoint(0, 1, 0, 0);
                else outServo = controlWaypoint(0);
                // if(this->jarak<jarakTemp){
                //     if(nearestRedBall.ymax > 0){
                //         if(nearestGreenBall.ymax > 0 && nearestRedBall.xmin>nearestGreenBall.xmin){
                //             outServo = controlBetween2Object(1, nearestRedBall.xmax, nearestGreenBall.xmin);
                //         }
                //         else{
                //             int tmp = (m+1)*nearestRedBall.xmin-m*nearestRedBall.xmax;
                //             outServo = controlBetween2Object(1, tmp, tmp);
                //         }
                //     }
                //     else if(nearestGreenBall.ymax > 0){
                //         int tmp = (m+1)*nearestGreenBall.xmax-m*nearestGreenBall.xmin;
                //         outServo = controlBetween2Object(1, tmp, tmp);
                //     }
                // }
                mtx5.unlock();
                emit sendOutput(speed, speed, outServo, outServo, servoKamera);
                usleep(100);
            }
            resetCameraControlPoint();
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
                        emit sendOutput(speedReverse, speedReverse, -50, -50, servoKamera);
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
        pidDist.p = pidTmp.p; pidDist.i = pidTmp.i; pidDist.d = pidTmp.d;
        qDebug()<<name<<" Finished";
    }
    return nextMissionIndex;
}

void SpeedChallenge::setNextDest(){
    ++wpIdx;
}

void SpeedChallenge::write(QJsonObject &obj) const {
    writeCommons(obj);
}