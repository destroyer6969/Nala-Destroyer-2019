#include "misi.h"

Misi::Misi()
{
    speed = 1500;
    arriveThreshold = 0.000013;
    running = 0;
    error = 0;
    errorSebelum = 0;
    memset(pid, 0, sizeof(pid));
    // init break conditions
    timeout = 0;
    breakWPIndex = -1;
    useCamera = false;
    lastWaypoint.latitude = lastWaypoint.longitude = startAngle = startAngle1 = startAngle2 = -1;
    xRangeLidar = yRangeLidar = -1;
    travelingSpeed = 1700;
    topLeftX = 120; topLeftY = 350;
    botLeftX = 0; botLeftY = 480;
    mirrorControlLine();
    yThreshold = 400;
    srfL=srfR=srfML=srfMR=srfS = 500;
    srfAvoidanceCounter[0]=srfAvoidanceCounter[1]=srfAvoidanceCounter[2]=0;
    srfMode = "11";
    speedReverse = 1400;
    servoKamera = 0;
    newImage = 0;
    memset(control, 0, sizeof(control));
}

void Misi::setObjAttr(const object_detection::ObjectAttr &msg){
    mtx1.lock();
    this->detectedObject = msg;
    newImage = 1;
    mtx1.unlock();
    // for(int i=0; i<1; i++){
    //     qDebug()<<detectedObject.detected_object[i].boxes.size()<<" <<size";
    //     for(int j=0; j<detectedObject.detected_object[i].boxes.size(); j++){
    //         qDebug() << detectedObject.detected_object[i].boxes[j].xmin;
    //         qDebug() << detectedObject.detected_object[i].boxes[j].ymin;
    //         qDebug() << detectedObject.detected_object[i].boxes[j].xmax;
    //         qDebug() << detectedObject.detected_object[i].boxes[j].ymax;
    //         qDebug() << detectedObject.detected_object[i].boxes[j].colorIdx << "\n=============\n";
    //     }
    //     qDebug()<<"\n"<<i;
    // }
}

void Misi::setSpeed(int speed){
    mtx3.lock();
    this->speed = speed;
    mtx3.unlock();
}

void Misi::setSpeedSlow(int newSpeed){
    mtx3.lock();
    this->speedSlow = newSpeed;
    mtx3.unlock();
}

void Misi::clearWaypoints(){
    waypoints.clear();
    NUM_WP = 0;
}

void Misi::deleteWaypoint(int idx){
    if((waypoints.size()) && (idx<waypoints.size())){
        mtx6.lock();
        waypoints.erase(waypoints.begin()+idx);
        NUM_WP--;
        mtx6.unlock();
    }
}

void Misi::setWaypoint(unsigned index, const PositionType& pos){
    mtx6.lock();
    if(index>=NUM_WP){
        waypoints.push_back(pos);
        NUM_WP++;
    }
    else
        this->waypoints[index] = pos;
    mtx6.unlock();
}

 void Misi::setDroneWaypoint(unsigned index, const PositionType& pos){

 }

void Misi::deleteDroneWaypoint(int index){
}

void Misi::setStatus(int state){
    this->active = state;
}

int Misi::controlWaypoint(int pidIdx, bool setControlStatus, int direction, bool distanceControl, bool anotherArrive){
    if(wpIdx>=waypoints.size())return 0;
    // direction 0 = default, direction 1 = always turn left, else always turn right
    if(setControlStatus){
        mtx10.lock();
        strcpy(control, "Waypoint");
        mtx10.unlock();
    }
    errorSebelum = error;
    sudutTujuan = (atan2(waypoints[wpIdx].latitude - currentPos.latitude,
                        currentPos.longitude - waypoints[wpIdx].longitude)
                    / radToDegrees) - 90;
    sudutTujuan = std::fmod((sudutTujuan + 360.0), 360.0);
    error  = std::fmod((sudutTujuan - compass + 360.0), 360.0);
    if(direction==0){
        if(error>180)error -= 360;
    }
    if(direction==1){
        error -= 360;
    }
    else if(direction==2 && error>350){
        error -= 360;
    }
    errorSudut = error;
    jarak       = sqrtf64(
                pow(waypoints[wpIdx].latitude - currentPos.latitude, 2)
                + pow(currentPos.longitude - waypoints[wpIdx].longitude, 2)
                );
    if(distanceControl/* && jarak>0.000018182*/){
        errorDistSebelum = errorDist;
        errorDist = distanceLineToPoint(startPos, waypoints[wpIdx], currentPos);
        controlDistanceOut = -(int)(pidDist.p*errorDist + pidDist.i*(errorDist+errorDistSebelum) + pidDist.d*(errorDist-errorDistSebelum));
        if(controlDistanceOut<-250)controlDistanceOut = -250;
        else if(controlDistanceOut>250)controlDistanceOut = 250;
    }
    else controlDistanceOut = 0;
    if(anotherArrive){
        if((lastWaypoint.longitude != waypoints[wpIdx].longitude) && (lastWaypoint.latitude != waypoints[wpIdx].latitude)){
            startAngle = sudutTujuan;
            startAngle1 = std::fmod((sudutTujuan + 270.0), 360.0);
            startAngle2 = std::fmod((sudutTujuan + 90.0), 360.0);
            lastWaypoint = waypoints[wpIdx];
        }
        else if((sudutTujuan<startAngle1 && sudutTujuan>startAngle2 && (startAngle<90 || startAngle>270)) || ((sudutTujuan>startAngle1 || sudutTujuan<startAngle2) && (startAngle>90 && startAngle<270))){
            startPos.latitude = waypoints[wpIdx].latitude;
            startPos.longitude = waypoints[wpIdx].longitude;
            setNextDest();
            emit arrived(wpIdx, startPos);
            emit updateSudutJarak(sudutTujuan, errorSudut, jarak);
            return (int)(pid[pidIdx].p*errorSudut + pid[pidIdx].i*(errorSudut+errorSebelum) + pid[pidIdx].d*(errorSudut-errorSebelum)) + controlDistanceOut;
        }
    }
    if(jarak<arriveThreshold){
        startPos.latitude = waypoints[wpIdx].latitude;
        startPos.longitude = waypoints[wpIdx].longitude;
        setNextDest();
        emit arrived(wpIdx, startPos);
    }
    emit updateSudutJarak(sudutTujuan, errorSudut, jarak);
    return (int)(pid[pidIdx].p*errorSudut + pid[pidIdx].i*(errorSudut+errorSebelum) + pid[pidIdx].d*(errorSudut-errorSebelum)) + controlDistanceOut;
}

int Misi::controlBetween2Object(int pidIdx, int x1, int x2){
    mtx10.lock();
    strcpy(control, "Between two object");
    mtx10.unlock();
    errorSebelum = error;
    float x = (float)(x1 + x2)/2.0;
    error = x - 320.0;
    this->controlPoint.setX((int)x);
    this->controlPoint.setY(240);
    return (int)(pid[pidIdx].p*error + pid[pidIdx].i*(error+errorSebelum) + pid[pidIdx].d*(error-errorSebelum));
}

int Misi::controlBoatSide(int pidIdx, int x, int y, int xTop, int yTop, int xBot, int yBot){
    if(xTop>xBot){
        mtx10.lock();
        strcpy(control, "Boat left side");
        mtx10.unlock();
    }
    else{
        mtx10.lock();
        strcpy(control, "Boat right side");
        mtx10.unlock();
    }
    errorSebelum = error;
    error = ((y - yTop) * (xBot - xTop) / (yBot - yTop)) + xTop - x;
    this->controlPoint.setX(x);
    this->controlPoint.setY(y);
    return (int)-(pid[pidIdx].p * error + pid[pidIdx].d * (error - errorSebelum) + pid[pidIdx].i * (error + errorSebelum));
}

int Misi::controlLidarAvoidance(int pidIdx, double yThreshold){
    if(xRangeLidar==-2)return 0;
    else if(xRangeLidar==-3)return 0;
    double xError;
    double yError = yThreshold - yRangeLidar;
    errorSebelum = error;
    if(xRangeLidar>0){
        xError = (double)30.0 - xRangeLidar;
        error = xError + yError;
    }
    else{
        xError = (double)-30.0 - xRangeLidar;
        error = xError - yError;
    }
    return (int)-(pid[pidIdx].p * error + pid[pidIdx].d * (error - errorSebelum) + pid[pidIdx].i * (error + errorSebelum));
}

int Misi::srfAvoidance(){
    if(srfL<100 || srfML<100 || srfMR<100 || srfR<100){
        ++srfAvoidanceCounter[0];
    }
    else if(srfL<200 || srfML<200){
        if(srfR>200 && srfMR>200){
            ++srfAvoidanceCounter[2];
            srfAvoidanceCounter[1]=srfAvoidanceCounter[0]=0;
        }
        else{
            ++srfAvoidanceCounter[0];
        }
    }
    else if(srfR<200 || srfMR<200){
        ++srfAvoidanceCounter[1];
        srfAvoidanceCounter[2]=srfAvoidanceCounter[0]=0;
    }
    else srfAvoidanceCounter[0]=srfAvoidanceCounter[1]=srfAvoidanceCounter[2]=0;
    if(srfAvoidanceCounter[0]>100){
        // qDebug()<<srfAvoidanceCounter[0]<<", "<<srfAvoidanceCounter[1]<<", "<<srfAvoidanceCounter[2];
        // qDebug()<<"srf mundur";
        mtx10.lock();
        strcpy(control, "SRF reverse");
        mtx10.unlock();
        return 0;
    }
    else if(srfAvoidanceCounter[1]>100){
        // qDebug()<<srfAvoidanceCounter[0]<<", "<<srfAvoidanceCounter[1]<<", "<<srfAvoidanceCounter[2];
        // qDebug()<<"srf kiri";
        mtx10.lock();
        strcpy(control, "SRF turn left");
        mtx10.unlock();
        return -500;
    }
    else if(srfAvoidanceCounter[2]>100){
        // qDebug()<<srfAvoidanceCounter[0]<<", "<<srfAvoidanceCounter[1]<<", "<<srfAvoidanceCounter[2];
        // qDebug()<<"srf kanan";
        mtx10.lock();
        strcpy(control, "SRF turn right");
        mtx10.unlock();
        return 500;
    }
    return -1;
}

int Misi::controlRange(int pidIdx, double targetRange, double currentRange, bool side){
    // Side == 0 for control left
    mtx10.lock();
    strcpy(control, "Range");
    mtx10.unlock();
    errorSebelum = error;
    error = targetRange-currentRange;
    if(side)error*=-1;
    return (int)(pid[pidIdx].p * error + pid[pidIdx].d * (error - errorSebelum) + pid[pidIdx].i * (error + errorSebelum));
}

object_detection::BoundingBox Misi::getNearestObject(int objectIdx, int colorIdx){
    object_detection::BoundingBox nearestObject;
    nearestObject.ymax = 0;
    bool objectNotFound = true;
    for(int i=0; i<detectedObject.detected_object[objectIdx].boxes.size(); i++){
        if(colorIdx==detectedObject.detected_object[objectIdx].boxes[i].colorIdx || colorIdx==-1){
            if(detectedObject.detected_object[objectIdx].boxes[i].ymax>nearestObject.ymax){
                objectNotFound = false;
                nearestObject = detectedObject.detected_object[objectIdx].boxes[i];
            }
        }
    }
    if (objectNotFound){
        nearestObject.xmin = nearestObject.xmax = nearestObject.ymin = nearestObject.ymax = -1;
    }
    return nearestObject;
}

object_detection::BoundingBox Misi::getNearestObject(){
    object_detection::BoundingBox nearestObject, temp;
    nearestObject.ymax = 0;
    for(int idx=0; idx<NUM_CLASSES; idx++){
        temp = getNearestObject(idx);
        if(temp.ymax>nearestObject.ymax) nearestObject = temp;
    }
    return temp;
}

object_detection::BoundingBox Misi::getNearestObstacle(int objectIdx, int yThreshold, int colorIdx){
    object_detection::BoundingBox nearestObject;
    nearestObject.ymax = -1;
    bool objectNotFound = true;
    for(int i=0; i<detectedObject.detected_object[objectIdx].boxes.size(); i++){
        int y = detectedObject.detected_object[objectIdx].boxes[i].ymax;
        if((y>yThreshold) && (colorIdx==detectedObject.detected_object[objectIdx].boxes[i].colorIdx || colorIdx==-1)){
            int x = (detectedObject.detected_object[objectIdx].boxes[i].xmax+detectedObject.detected_object[objectIdx].boxes[i].xmin)/2;
            if (x < 320 && ((y-topLeftY)*(botLeftX-topLeftX)/(botLeftY-topLeftY)+botLeftX-detectedObject.detected_object[objectIdx].boxes[i].xmax)<0) {
                if(detectedObject.detected_object[objectIdx].boxes[i].ymax>nearestObject.ymax){
                    nearestObject = detectedObject.detected_object[objectIdx].boxes[i];
                }
                objectNotFound = false;
            }
            else if (x >= 320 && ((y-topRightY)*(botRightX-topRightX)/(botRightY-topRightY)+botRightX-detectedObject.detected_object[objectIdx].boxes[i].xmin)>0) {
                if(detectedObject.detected_object[objectIdx].boxes[i].ymax>nearestObject.ymax){
                    nearestObject = detectedObject.detected_object[objectIdx].boxes[i];
                }
                objectNotFound = false;
            }
        }
    }
    if (objectNotFound){
        nearestObject.xmin = nearestObject.xmax = nearestObject.ymin = nearestObject.ymax = -1;
    }
    return nearestObject;
}

object_detection::BoundingBox Misi::getNearestObstacle(int yThreshold){
    object_detection::BoundingBox nearestObject, temp;
    nearestObject.ymax = -2;
    for(int idx=0; idx<NUM_CLASSES; idx++){
        temp = getNearestObstacle(idx, yThreshold);
        if(temp.ymax>nearestObject.ymax){
            nearestObject = temp;
        }
    }
    return nearestObject;
}

void Misi::setNextMission(int index){
    nextMissionIndex = index;
}

QString Misi::getName(){
    return this->name;
}

void Misi::setP1(double val){
    mtx3.lock();
    this->pid[0].p = val;
    mtx3.unlock();
}
void Misi::setI1(double val){
    mtx3.lock();
    this->pid[0].i = val;
    mtx3.unlock();
}
void Misi::setD1(double val){
    mtx3.lock();
    this->pid[0].d = val;
    mtx3.unlock();
}
void Misi::setP2(double val){
    mtx3.lock();
    this->pid[1].p = val;
    mtx3.unlock();
}
void Misi::setI2(double val){
    mtx3.lock();
    this->pid[1].i = val;
    mtx3.unlock();
}
void Misi::setD2(double val){
    mtx3.lock();
    this->pid[1].d = val;
    mtx3.unlock();
}
void Misi::setP3(double val){
    mtx3.lock();
    this->pid[2].p = val;
    mtx3.unlock();
}
void Misi::setI3(double val){
    mtx3.lock();
    this->pid[2].i = val;
    mtx3.unlock();
}
void Misi::setD3(double val){
    mtx3.lock();
    this->pid[2].d = val;
    mtx3.unlock();
}
void Misi::setP4(double val){
    mtx3.lock();
    this->pid[3].p = val;
    mtx3.unlock();
}
void Misi::setI4(double val){
    mtx3.lock();
    this->pid[3].i = val;
    mtx3.unlock();
}
void Misi::setD4(double val){
    mtx3.lock();
    this->pid[3].d = val;
    mtx3.unlock();
}
void Misi::setPD(double val){
    mtx3.lock();
    this->pidDist.p = val;
    mtx3.unlock();
}
void Misi::setID(double val){
    mtx3.lock();
    this->pidDist.i = val;
    mtx3.unlock();
}
void Misi::setDD(double val){
    mtx3.lock();
    this->pidDist.d = val;
    mtx3.unlock();
}

PositionType& Misi::getWaypoint(unsigned index){
    return this->waypoints[index];
}

int Misi::getNumWaypoints(){
    return this->waypoints.size();
}

bool Misi::isRunning(){
    return running;
}

void Misi::stop(){
    mtx.lock();
    this->running = 0;
    mtx.unlock();
}

void Misi::setTimeout(double timeout){
    this->timeout = timeout*60;
}

void Misi::setBreakWPIndex(int idx){
    breakWPIndex = idx;
}

void Misi::setUseCamera(int cek){
    useCamera = cek;
}

void Misi::writeCommons(QJsonObject &obj) const{
    // pid (2)
    QJsonArray pidArr;
    for(int i=0; i<3; i++){
        QJsonObject pidObj;
        pidObj["p"] = pid[i].p;
        pidObj["i"] = pid[i].i;
        pidObj["d"] = pid[i].d;
        pidArr.append(pidObj);
    }
    obj["pid"] = pidArr;

    // speed
    obj["speed"] = speed;

    // break conditions
    obj["timeout"] = timeout;
    obj["breakwp"] = breakWPIndex;
    obj["useCam"] = useCamera;
}

int Misi::squaredPixelDistance(int x1, int y1, int x2, int y2){
    int x = x2-x1;
    int y = y2-y1;
    return x*x+y*y;
}

void Misi::setControlLine(int topLeftX, int topLeftY, int botLeftX, int botLeftY, int topRightX, int topRightY, int botRightX, int botRightY){
    QMutex mtx7;
    mtx7.lock();
    this->topLeftX = topLeftX; this->topLeftY = topLeftY;
    this->botLeftX = botLeftX; this->botLeftY = botLeftY;

    this->topRightX = topRightX; this->topRightY  = topRightY;
    this->botRightX = botRightX; this->botRightY = botRightY;
    mtx7.unlock();
}

void Misi::setLidarObstacle(double xRange, double yRange){
    mtx2.lock();
    xRangeLidar = xRange;
    yRangeLidar = yRange;
    mtx2.unlock();
}

QPoint Misi::getCameraControlPoint(){
    return this->controlPoint;
}

void Misi::resetCameraControlPoint(){
    mtx4.lock();
    this->controlPoint.setX(-6);
    this->controlPoint.setY(-6);
    mtx4.unlock();
}

void Misi::setSRF(int srfL_, int srfML_, int srfMR_, int srfR_, int srfS_){
    mtx8.lock();
    srfL = srfL_;
    srfR = srfR_;
    srfML = srfML_;
    srfMR = srfMR_;
    srfS = srfS_;
    mtx8.unlock();
}

void Misi::setLidarLeftObstacle(double range){
    mtx9.lock();
    lidarLeftObstacle = range;
    mtx9.unlock();
}

int Misi::getControlLine(int i){
    if(i==0){
        return topLeftX;
    }
    else if(i==1){
        return topLeftY;
    }
    else if(i==2){
        return botLeftX;
    }
    else if(i==3){
        return botLeftY;
    }
    else if(i==4){
        return topRightX;
    }
    else if(i==5){
        return topRightY;
    }
    else if(i==6){
        return botRightX;
    }
    else if(i==7){
        return botRightY;
    }
}

int Misi::getYThreshold(){
    return yThreshold;
}

void Misi::setLidarControlRange(int val){
    mtx3.lock();
    lidarControlRange = val;
    mtx3.unlock();
}

void Misi::mirrorControlLine(int side){
    // default: 0 == mirror from left line
    if(side==0){
        topRightX = 640-topLeftX; topRightY = topLeftY;
        botRightX = 640-botLeftX; botRightY = botLeftY;
    }
    else{
        topLeftX = 640-topRightX; topLeftY = topRightY;
        botLeftX = 640-botRightX; botLeftY = botRightY;
    }
}

double Misi::distanceLineToPoint(PositionType p1, PositionType p2, PositionType p){
    double y2Miny1 = p2.latitude-p1.latitude;
    double x2Minx1 = p2.longitude-p1.longitude;
    return ((y2Miny1*p.longitude - x2Minx1*p.latitude + p2.longitude*p1.latitude - p2.latitude*p1.longitude)
            / sqrt(y2Miny1*y2Miny1 + x2Minx1*x2Minx1)) * 11000000.0;
}

void Misi::start(){
    double tmp1, tmp2;
    tmp1 = currentPos.latitude;
    tmp2 = currentPos.longitude;
    usleep(10000);
    while(running && tmp1==currentPos.latitude && tmp2==currentPos.longitude)usleep(100);
    startPos.latitude = currentPos.latitude;
    startPos.longitude = currentPos.longitude;
    emit arrived(0, startPos);
}

void Misi::setYThreshold(int val){
    mtx3.lock();
    yThreshold = val;
    mtx3.unlock();
}

void Misi::setServoMuterMax(int val){
    mtx3.lock();
    servoMuterMax = val;
    mtx3.unlock();
}

char* Misi::getControl(){
    return control;
}

void Misi::setUseSomething(int val){
    mtx3.lock();
    useSomething = val;
    mtx3.unlock();
}

void Misi::setUseSomething2(int val){
    mtx3.lock();
    useSomething2 = val;
    mtx3.unlock();
}
