#include "circumnav.h"

CircumNav::CircumNav()
{
    init();
    clearWaypoints();
}

CircumNav::~CircumNav(){

}

void CircumNav::init(){
    NUM_WP = 0;
    speed       = 1500;
    name = "Circumnavigation";
}

int CircumNav::startMission(){
    if(active){
        
    }
    return nextMissionIndex;
}

void CircumNav::setNextDest(){
    
}

void CircumNav::write(QJsonObject &obj) const{
    writeCommons(obj);
}
