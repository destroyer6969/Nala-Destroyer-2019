#include "lidar.h"

Lidar::Lidar(QObject *parent)
    :QObject(parent)
{
    radToDegrees = M_PI/(double)180.0;
    longitudeConst[REED_CANAL_PARK] = 9.771;
    longitudeConst[DANAU_8] = 11.03;
    latitudeConst = 11.12;
    location = DANAU_8;
    range = 800.0; angle = 0.0;
    angle = 0;
    map = NULL;
    running = 0;
    lidarConnected = 0;
    miring = 1;
    portNum = -1;
    nhCreated = 0;
    std::cout<<scanMode;
}

void Lidar::run(std::vector<std::string> *portNames){
    if(!lidarConnected)if(connectLidar(portNames))return;
    startMotor();
    RplidarScanMode current_scan_mode;
    scanMode = "Boost";
    if(scanMode.empty()){op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    }
    else{
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);
        std::cout<<scanMode<<std::endl;
        if(IS_OK(op_result)){
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scanMode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }
            if (selectedScanMode == _u16(-1)){
                ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:", scanMode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                            iter->max_distance, (1000/iter->us_per_sample));
                }
                op_result = RESULT_OPERATION_FAIL;
            }
            else{
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }
    qDebug() << "Lidar running";
    running = 1;
    bool obstacleDetected, reversed;
    double angle2 = 0;
    while(map==NULL);
    QPainter painter(map);
    QPen toscaPen(QColor(tosca), 1);
    QPen redPen(Qt::red, 0);
    unsigned char data[1000];
    ros::Time startScanTime;
    ros::Time endScanTime;
    double scanDuration, timeIncrement;
    float angleMax, angleMin, angleIncrement;
    int nodeCount;
    while(running){
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);
        op_result = drv->grabScanDataHq(nodes, count);
        if(op_result == RESULT_OK){
            op_result = drv->ascendScanData(nodes, count);
            if(op_result == RESULT_OK){
                int startNode = 0, endNode = 0;
                int i = 0;
                block = 0;
                // find the first valid node and last valid node
                while (nodes[i++].dist_mm_q2 == 0);
                startNode = i-1;
                i = count -1;
                while (nodes[i--].dist_mm_q2 == 0);
                endNode = i+1;
                angleMin = getAngle(nodes[startNode]) * radToDegrees;
                angleMax = getAngle(nodes[endNode]) * radToDegrees;
                nodeCount = endNode-startNode+1;
                timeIncrement = scanDuration/double(nodeCount-1);
                angleIncrement = (angleMax-angleMin)/(double)(nodeCount-1);
                nearestXRange = nearestYRange = nearestLeftObstacle = maxDistance;
                for(size_t i=0; i<nodeCount; i++){
                    obstacleDetected = true;
                    angleFromBoat = (angleMin + angleIncrement * i) / radToDegrees;
                    if(angleFromBoat>100 && angleFromBoat<260)continue;
                    angle = (compass + angleFromBoat - miring) * radToDegrees;
                    range = (double)nodes[i].dist_mm_q2/4.0f/10;
                    obstaclePos.latitude = this->currentPos.latitude + (cos(angle) * range / latitudeConst * 0.000001);
                    obstaclePos.longitude = this->currentPos.longitude + (sin(angle) * range / longitudeConst[location] * 0.000001);
                    obstaclePoint = calculatePoint(obstaclePos);
                    currentPoint = calculatePoint(currentPos);
                    painter.setPen(toscaPen);
                    painter.drawLine(obstaclePoint, currentPoint);
                    painter.setPen(redPen);
                    painter.setBrush(Qt::red);
                    if(range>maxDistance){
                        obstacleDetected = false;
                        range = maxDistance;
                    }
                    else if(range<15.0){
                        obstacleDetected = false;
                        range = 0;
                    }
                    else{
                        painter.drawEllipse(obstaclePoint, 1, 1);
                        yRange = range*sin(angleFromBoat);
                        xRange = range*cos(angleFromBoat);
                        if(yRange<20 && (xRange>-30 || xRange<30)){
                            if(block==0){
                                if(xRange<0)block = -1;
                                else block = 1;
                            }
                            else if(block==-1 && xRange>0){
                                block = 3;
                            }
                            else if(block==1 && xRange<0){
                                block = 3;
                            }
                            if(yRange<nearestYRange){
                                nearestXRange = xRange;
                                nearestYRange = yRange;
                            }
                        }
                        if(yRange<50 && xRange<0 && xRange<nearestLeftObstacle)nearestLeftObstacle=xRange;
                    }
                }
                if(block==3 || nearestYRange<20){
                    if(nearestXRange>0)emit obstacle(-2,-2);
                    else emit obstacle(-3,-3);
                    
                }
                else if(nearestXRange != maxDistance){
                    emit obstacle(nearestXRange, nearestYRange);
                }
                else emit obstacle(-1, -1);
                emit leftObstacle(nearestLeftObstacle);
            }
        }
        usleep(1);
    }
    stopMotor();
    qDebug() << "Lidar stopped";
}

void Lidar::getCurrentPos(double latitude, double longitude, double compass){
    this->currentPos.latitude = latitude;
    this->currentPos.longitude = longitude;
    this->compass = compass;
}

QPoint Lidar::calculatePoint(const PositionType& pos){
    return QPoint((int)((pos.longitude-longitude_awal)*skala_longitude), (int)((pos.latitude-latitude_awal)*skala_latitude));
}

void Lidar::getLocation(double latitude_, double longitude_, double skala_latitude_, double skala_longitude_){
    this->latitude_awal = latitude_;
    this->longitude_awal = longitude_;
    this->skala_latitude = skala_latitude_;
    this->skala_longitude = skala_longitude_;
}

void Lidar::initImageMap(){
    emit requestImage();
}

void Lidar::getImageMap(QPixmap *map_){
    this->map = map_;
}

void Lidar::stop(){
    this->running = 0;
    portNum = -1;
}

Lidar::~Lidar(){
    if(running){
        drv->stop();
        drv->stopMotor();
        RPlidarDriver::DisposeDriver(drv);
    }
    if(nhCreated)delete nh;
}

bool Lidar::getRPLIDARDeviceInfo(RPlidarDriver * drv){
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            ROS_ERROR("Error, unexpected error, code: %x",op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    ROS_INFO("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}

bool Lidar::checkRPLIDARHealth(RPlidarDriver * drv){
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
        ROS_INFO("RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}

void Lidar::stopMotor(){
    if(!drv)return;
    qDebug()<<"Stop lidar motor";
    drv->stop();
    drv->stopMotor();
}

void Lidar::startMotor(){
    if(!drv)return;
    qDebug()<<"Start lidar motor";
    drv->startMotor();
    drv->startScan(0,1);
}

float Lidar::getAngle(const rplidar_response_measurement_node_hq_t& node){
    return node.angle_z_q14 * 90.f / 16384.f;
}

bool Lidar::connectLidar(std::vector<std::string> *portNames){
    auto it = portNames->begin();
    int a = -1;
    std::string serialPort;
    std::string frameID;
    maxDistance = 800.0;
    bool inverted = false;
    angleCompensate = false;
    angleCompensateMultiple = 1;
    int serialBaudrate = 256000;
    char tmp[30];
    if(nhCreated)delete nh;
    nh = new ros::NodeHandle("~");
    nhCreated = 1;
    for(it; it!=portNames->end(); it++){
        ++a;
        strcpy(tmp, std::string(*it).c_str());
        tmp[strlen(tmp)-1]=0;
        if(strcmp(tmp, "/dev/ttyUSB")!=0)continue;
        nh->param<std::string>("serial_port", serialPort, std::string(*it).c_str());
        nh->param<int>("serial_baudrate", serialBaudrate, 256000);
        nh->param<std::string>("frame_id", frameID, "laser_frame");
        nh->param<bool>("inverted", inverted, false);
        nh->param<bool>("angle_compensate", angleCompensate, false);
        nh->param<std::string>("scan_mode", scanMode, std::string());
        drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
        if(!drv){
            continue;
        }
        if (IS_FAIL(drv->connect(serialPort.c_str(), (_u32)serialBaudrate))) {
            RPlidarDriver::DisposeDriver(drv);
            continue;
        }
        if (!getRPLIDARDeviceInfo(drv))continue;
        if (!checkRPLIDARHealth(drv)) {
            RPlidarDriver::DisposeDriver(drv);
            continue;
        }
        printf("Port \"%s\" connected to lidar\n", std::string(*it).c_str());
        portNum = a;
        break;
    }
    if(portNum<0){
        if(!drv){
            qDebug()<<"Create Driver fail, exit";
            portNum = -2;
            return 1;
        }
        if (IS_FAIL(drv->connect(serialPort.c_str(), (_u32)serialBaudrate))) {
            qDebug()<<"Error, cannot bind to the specified serial port "<<serialPort.c_str();
            RPlidarDriver::DisposeDriver(drv);
            portNum = -2;
            return 1;
        }
        if (!getRPLIDARDeviceInfo(drv)){
            portNum = -2;
            return 1;
        }
        if (!checkRPLIDARHealth(drv)) {
            RPlidarDriver::DisposeDriver(drv);
            portNum = -2;
            return 1;
        }
    }
    return 0;
}

bool Lidar::isRunning(){
    return running;
}

int Lidar::getPortNum(){
    return portNum;
}
