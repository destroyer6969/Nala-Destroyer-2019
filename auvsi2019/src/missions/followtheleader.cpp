#include "followtheleader.h"

FollowTheLeader::FollowTheLeader()
{
    init();
    clearWaypoints();
}

FollowTheLeader::~FollowTheLeader(){

}

void FollowTheLeader::init(){
    NUM_WP = 0;
    speed       = 1500;
    flagColor   = 0;
    statusMission= "Waiting";
    name = "Follow The Leader";
}


int FollowTheLeader::startMission(){
    if(active){
        
    }
    return nextMissionIndex;
}

void FollowTheLeader::setNextDest(){
    
}

void FollowTheLeader::write(QJsonObject &obj) const{
    writeCommons(obj);

    obj["speed2"] = speed2;
}
