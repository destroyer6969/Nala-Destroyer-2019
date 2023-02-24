#include "maintainheading.h"

MaintainHeading::MaintainHeading()
{
    init();
    clearWaypoints();
}

MaintainHeading::~MaintainHeading(){

}

void MaintainHeading::init(){
    NUM_WP = 0;
    speed       = 1500;
    sudut       = 0;
    name = "Maintain Heading";
}


void MaintainHeading::setSudut(int sudut){
    this->sudut  = sudut;
}

int MaintainHeading::startMission(){
    if(active){
        
    }
    return nextMissionIndex;
}

void MaintainHeading::setNextDest(){
    
}

void MaintainHeading::write(QJsonObject &obj) const{
    writeCommons(obj);

    obj["sudut"] = sudut;
}
