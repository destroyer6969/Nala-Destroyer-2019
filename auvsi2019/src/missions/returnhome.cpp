#include "returnhome.h"

ReturnHome::ReturnHome()
{
    init();
    clearWaypoints();
}

ReturnHome::~ReturnHome(){

}

void ReturnHome::init(){
    NUM_WP = 0;
    speed       = 1500;
    name = "Return Home";

}


int ReturnHome::startMission(){
    if(active && waypoints.size()){
        qDebug()<<name<<" Started";
        running = 1;
        wpIdx = 0;
        start();
        int outSpeed;
        while(running && wpIdx!=breakWPIndex && wpIdx<waypoints.size()){
            // if(xRangeLidar!=-1) outServo = controlLidarAvoidance(2, 150);
            outSpeed = travelingSpeed;
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
        qDebug()<<name<<" Finished";
    }
    return nextMissionIndex;
}

void ReturnHome::setNextDest(){
    ++wpIdx;
}

void ReturnHome::write(QJsonObject &obj) const{
    writeCommons(obj);
}
