#include "tcpconnection.h"

TCPConnection::TCPConnection(QObject *parent):QObject(parent){
    courseName[0] = "danau8";
    courseName[1] = "courseA";
    courseName[2] = "courseB";
    courseName[3] = "courseC";
    courseName[4] = "courseD";
    courseName[5] = "robotika";
    missionName[0] = "null";
    missionName[1] = "gate";
    missionName[2] = "speed";
    missionName[3] = "docking";
    missionName[4] = "raise";
    missionName[5] = "path";
    missionName[6] = "null";
    missionName[7] = "return";
    misiIdx = 0;
    courseIdx = 5;
    serverNotConnected = 1;
    missionNotStarted = 1;
    timerHeartBeat = new QTimer(0);
    timerHeartBeat->setInterval(1);
    timerHeartBeat->start();
    latitude = longitude = 0;
    sysMode = 1;
    time(&t1);
}

TCPConnection::~TCPConnection(){
    closeConnection();
    delete timerHeartBeat;
    qDebug()<<"TCP Thread destroyed";
}

void TCPConnection::getCurrentPos(double newLatitude, double newLongitude){
    latitude = newLatitude;
    longitude = newLongitude;
}

void TCPConnection::openConnection(const char *ip, int port){
    // ip = "127.0.0.1";
    qDebug()<<port;
    /*
    for(int x=0; x<5; x++){
        char command[28], buff[BUFSIZ];
        sprintf(command, "ping -c 1 %s", ip);
        FILE* stream = popen(command, "r");
        fread(buff, BUFSIZ, 1, stream);
        if (strstr(buff, "1 received") == NULL){
            emit sendLabel("[ICMP] Ping failed");
            if(x==4)return;
        }
        else{
            emit sendLabel("[ICMP] Ping success");
            break;
        }
    }
    */
    qDebug()<<"test1";
    if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        qDebug()<<"Socket creation error"; 
        return;
    }
    qDebug()<<"test";
    memset(&serverAddress, '0', sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    if(inet_pton(AF_INET, ip, &serverAddress.sin_addr)<=0){ 
        qDebug()<<"Invalid IP address/ IP Address not supported";
        return;
    }
    qDebug()<<"test3";
    if(::connect(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0){ 
        emit sendLabel("[FAILED] Cannot connect to Server");
        return;
    }
    qDebug()<<"test2";
    emit sendLabel("Connected to Server");
    serverNotConnected = 0;
}

void TCPConnection::startConnection(){
    if(serverNotConnected) {
        emit sendLabel("[FAILED] Can't start the course.");
        return;
    }
    // char buffer[100];
    // int checkSum;
    // buffer[0] = 0;
    // sprintf(buffer, "$SVSTR,%s,BARUNASTRA", courseName[courseIdx]);
    // checkSum = calcCheckSum(buffer);
    // sprintf(buffer, "%s*%02X", buffer, checkSum);
    // send(sock, buffer, strlen(buffer), 0);
    // read(sock, buffer, 100);
    // if(buffer[0]=0 && isResponseOK(buffer)){
    //     emit sendLabel("[OK] Start the course");
    // }
    // else{
    //     emit sendLabel("[FAILED] Can't start the course.");
    // }
    // qDebug()<<buffer;
    emit sendLabel("[OK] Start the course");
    missionNotStarted=0;
}

void TCPConnection::stopConnection(){
    if(missionNotStarted)return;
    // char buffer[100];
    // int checkSum;
    // sprintf(buffer, "$SVEND,%s,BARUNASTRA", courseName[courseIdx]);
    // checkSum = calcCheckSum(buffer);
    // sprintf(buffer, "%s*%02X", buffer, checkSum);
    // send(sock, buffer, strlen(buffer), 0);
    // read(sock, buffer, 100);
    // if(isResponseOK(buffer)){
    //     qDebug() << "close OK";
    // }
    // qDebug()<<buffer;
    missionNotStarted = 1;
}

void TCPConnection::closeConnection(){
    if(!serverNotConnected){
        int t = 1;
        setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&t,sizeof(int));
        close(sock);
        serverNotConnected = 1;
        missionNotStarted = 1;
    }
}

void TCPConnection::heartBeat(){
    if(missionNotStarted || (time(NULL)-t1)<1) return;
    // if((time(NULL)-t1)<1) return;
    time(&t1);
    t = time(NULL);
    tm = *localtime(&t);
    char buffer[128];
    char ns, ew;
    if(latitude<0){
        ns = 'S';
        latitude *= -1;
    }
    else
    {
        ns = 'N';
    }

    if(longitude<0){
        ew = 'W';
        longitude *= -1;
    }

    else{
        ew = 'E';
    }
    
    sprintf(buffer, "$RBHRB,%02d%02d%02d,%02d%02d%02d,%.05lf,%c,%.05lf,%c,%s,%d",
    tm.tm_mday,  tm.tm_mon+1, tm.tm_year%100, tm.tm_hour, tm.tm_min, tm.tm_sec, latitude, ns, longitude, ew, teamID, sysMode);
    // sprintf(buffer, "$SVHRT,%s,BARUNASTRA,%.4d%.2d%.2d%.2d%.2d%.2d,%s,%.6f,%.6f",
    // courseName[courseIdx], tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, missionName[misiIdx], latitude, longitude);
    int c=0;
    for(int q=strlen(buffer)-7; c!=3; q--){
        if(c==0){
            if(buffer[q]=='W' || buffer[q]=='E'){
                q-=7;
                buffer[q]='.';
                c=1;
            }
        }
        else if(c==1){
            if(buffer[q]=='N' || buffer[q]=='S'){
                q-=7;
                buffer[q]='.';
                c=3;
            }
        }
    }
    
    int checkSum = calcCheckSum(buffer);
    sprintf(buffer, "%s*%02X\n", buffer, checkSum);
    send(sock , buffer , strlen(buffer) , 0 );
    qDebug()<<"Sent:"<<buffer;
    memset(buffer, 0, sizeof(buffer));
    read(sock , buffer, 128);
    qDebug()<<"Received:"<<buffer;
    
    if(isResponseOK(buffer)){
        char string_time[9];
        sprintf(string_time, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
        emit sendLabel("[OK] Heartbeat at " + QString(string_time) +" sent");
    }
    else{
        emit sendLabel("[ERR] Got error  message from TD.");
    }
    
    if(flag){
        usleep(1000);
        sendFlagMsg();
        qDebug()<<"========FLAAAGG=======";
    }
    if(dock){
        usleep(1000);
        sendDockMsg();
        qDebug()<<"========DDDOOOOCCCKKK=======";
    }
}

int TCPConnection::calcCheckSum(char* buffer){
    int checkSum = buffer[1];
    for(int i=2; i<strlen(buffer); i++){
        if(buffer[i]=='*'){
            break;
        }
        checkSum^=buffer[i];
    }
    return checkSum;
}

bool TCPConnection::isResponseOK(char* buffer){
    if(strstr("TDERR", buffer) == NULL)
        return true;
    else
        return false;
}

void TCPConnection::setCourse(int courseIdx_){
    courseIdx = courseIdx_;
}

void TCPConnection::setMission(int misiIdx_){
    misiIdx = misiIdx_;
}

void TCPConnection::sendDockMsg(){
    t = time(NULL);
    tm = *localtime(&t);
    char buffer[128];
    sprintf(buffer, "$RBDOK,%02d%02d%02d,%02d%02d%02d,%s,%d",
    tm.tm_mday, tm.tm_mon+1, tm.tm_year%100, tm.tm_hour, tm.tm_min, tm.tm_sec, teamID, dock);
    
    int checkSum = calcCheckSum(buffer);
    sprintf(buffer, "%s*%02X\n", buffer, checkSum);
     send(sock , buffer , strlen(buffer) , 0 );
     qDebug()<<"Sent:"<<buffer;
     memset(buffer, 0, sizeof(buffer));
     read(sock , buffer, 128);
     qDebug()<<"Received:"<<buffer;
    
    if(isResponseOK(buffer)){
        char string_time[9];
        sprintf(string_time, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
        emit sendLabel("[OK] Heartbeat at " + QString(string_time) +" sent");
        dock = 0;
    }
    else{
        emit sendLabel("[ERR] Got error  message from TD.");
    }
}

void TCPConnection::sendFlagMsg(){
    t = time(NULL);
    tm = *localtime(&t);
    char buffer[128];
    sprintf(buffer, "$RBFLG,%02d%02d%02d,%02d%02d%02d,%s,%d",
    tm.tm_mday, tm.tm_mon+1, tm.tm_year%100, tm.tm_hour, tm.tm_min, tm.tm_sec, teamID, flag);
    
    int checkSum = calcCheckSum(buffer);
    sprintf(buffer, "%s*%02X\n", buffer, checkSum);
    send(sock , buffer , strlen(buffer) , 0 );
    qDebug()<<"Sent:"<<buffer;
    memset(buffer, 0, sizeof(buffer));
    read(sock , buffer, 128);
    qDebug()<<"Received:"<<buffer;
    
    if(isResponseOK(buffer)){
        char string_time[9];
        sprintf(string_time, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
        emit sendLabel("[OK] Heartbeat at " + QString(string_time) +" sent");
        flag = 0;
    }
    else{
        emit sendLabel("[ERR] Got error  message from TD.");
    }
}

void TCPConnection::setSysMode(int mode){
    QMutex mtxx;
    mtxx.lock();
    sysMode = mode;
    // qDebug()<<sysMode;
    mtxx.unlock();
}

void TCPConnection::setFlagNumber(int val){
    mtxx.lock();
    flag = val;
    mtxx.unlock();
}

void TCPConnection::setDockNumber(int val){
    mtxx.lock();
    dock = val;
    mtxx.unlock();
}
