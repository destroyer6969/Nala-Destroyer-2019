#include "slalom.h"

Slalom::Slalom()
{
    init();
    clearWaypoints();
}

Slalom::~Slalom(){

}

void Slalom::init(){
    NUM_WP = 0;
    speed       = 1500;
    name = "Slalom";
}


int Slalom::startMission(){
    if(active){
        
    }
    return nextMissionIndex;
}

void Slalom::setNextDest(){
    
}

void Slalom::write(QJsonObject &obj) const{
    writeCommons(obj);
}
