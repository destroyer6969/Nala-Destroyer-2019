#include "gpsconnection.h"
#include <QDebug>
#include <QMutex>


/////////////// DUMMDUMM /////////////////////
void GPSConnection::run(std::vector<std::string>*portNames){
	//29,22176710, -81,0267149
	if(!opened){
		QMutex m;
		auto it=portNames->begin();
		int a=0;
        char tmp[30], tmp_[30], cmd[30], baud[10];
        FILE *stream;
		portNum = -1;
		for(it; it!=portNames->end(); it++){
            memset(baud, 0, sizeof(baud));
            strcpy(tmp, std::string(*it).c_str());
            strcpy(cmd, "stty -F ");
            strcat(cmd, std::string(*it).c_str());
            tmp[strlen(tmp)-1]=0;
            if(strcmp(tmp, "/dev/ttyACM")!=0)continue;
            stream = popen(cmd, "r");
            fgets(tmp_, 30, stream);
            int spaceCounter=0, qq=0;
            for(int q=2; q<strlen(tmp_); q++){
                if(tmp_[q]==' '){
					++spaceCounter;
					continue;
				}
                if(spaceCounter==1){
                    baud[qq]=tmp_[q];
                    ++qq;
                }
                if(spaceCounter==2)break;
            }
            pclose(stream);
            // qDebug()<<baud<<"test";
            // if(strcmp(baud,baudRate)!=0)continue;
			if(init_serial_port(std::string(*it).c_str()) != -1){
				time(&startTime);
				while((time(NULL)-startTime)<2 && portNum==-1){
					result = read(fd, &cp, 1);
					if(result>0) mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
					if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT){
						mavlink_msg_heartbeat_decode(&message, &heartBeat);
						if(heartBeat.type == 2){
							printf("Port \"%s\" connected to boat's Pixhawk\n", std::string(*it).c_str());
							m.lock();
							opened = true;
							portNum = a;
							m.unlock();
							break;
						}
					}
					usleep(1);
				}
			}
			if(opened)break;
			++a;
		}
		if(!opened){
			printf("Error: cannot connect to boat's pixhawk\n");
			m.lock();
			portNum = -2;
			m.unlock();
		}
	}
	int i=0;
	isKilled = false;
	ros::Rate rate(40);
	//emit sendCurrentPos(-7.28724553, 112.7960103, 45.0);la+=0.0000001, li+=0.0000001
	double la=29.1515742, li=-81.01697097, co=0.0;
	double compass;
	// startTime = clock();
	while(isKilled==false){
		/*
		emit sendCurrentPos(la+=0.0000001, li, co);
		emit sendCurrentPos(la, li);
		rate.sleep();
		*/
		result = read(fd, &cp, 1);
		if(result>0) mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
		if(message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT){
			mavlink_msg_global_position_int_decode(&message, &global_pos);
			lattDouble = (double)global_pos.lat/(double)10000000;
			longDouble = (double)global_pos.lon/(double)10000000;
			compass = (double)global_pos.hdg/(double)100;
			if(lattDouble==0)continue;
			if(longDouble==0)continue;
			if(compass==0)continue;
			emit sendCurrentPos(lattDouble, longDouble, compass);
			emit sendCurrentPos(lattDouble, longDouble);
		}
		else if(message.msgid == MAVLINK_MSG_ID_ATTITUDE){
			mavlink_msg_attitude_decode(&message, &attitude);
			emit sendAttitude(attitude.roll/radToDegrees);
		}
		
		usleep(1);
	}
}

void GPSConnection::runFromPhone(){
	// 40:B8:37:1F:B6:82 sony xperia ku
	if(init_bluetooth("/dev/rfcomm0")==-1)qDebug()<<"Cannot connect bluetooth";
	portNum = 9;
	int cnt;
	char data[10][50];
	double compass;
	isKilled = false;
	while (isKilled==false && fd!=-1)
	{
		cnt=0;
		read(fd,inBuffer,100);
		char *inBufferOut = strtok(inBuffer,delim);
		memset(data, 0, sizeof(data));
		while(inBufferOut!=NULL)
		{
			strcpy(data[cnt],inBufferOut);
			//printf("%d: %s\n",cnt, data[cnt]);
			inBufferOut= strtok(NULL,delim);
			++cnt;
		}
		// for(int i=0;i<=cnt;i++){
		// 	printf("%d: %s\n",i, data[i]);
		// }
		if(data[0][1]=='2'){
			xGaussData = atof(data[2])*0.48828125;
			yGaussData = atof(data[3])*0.48828125;
			D = atan2(-xGaussData,yGaussData)*(180/M_PI);
			if(D<0){
				D=D+360;
			}
			if(D!=0)compass = D;
			printf("%lf\n",D);
		}
		if(data[0][1]=='9' && data[0][2]=='8'){
//			data[2][2]=',';
//			data[3][3]=',';
			latitude = atof(data[2]);
			longitude = atof(data[3]);
			printf("%s,%s\n",data[2],data[3]);
			if(latitude!=0)lattDouble = latitude;
			if(longitude!=0)longDouble = longitude;
		}
		emit sendCurrentPos(lattDouble, longDouble, compass);
		emit sendCurrentPos(lattDouble, longDouble);
		usleep(1);
	}
}

void GPSConnection::stop(){
	isKilled = true;
	if(close(fd))qDebug()<<"Error Closing Pixhawk Port";
	else{
		qDebug()<<"Pixhawk disconnected";
	}
	opened = 0;
}

GPSConnection::GPSConnection(QObject *parent): QObject(parent){
    isKilled = true;
	opened = 0;
	radToDegrees = M_PI/(double)180.0;
	portNum = -1;
	baudRate = "57600";
}

int GPSConnection::init_serial_port(const char *port_name){
	fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)return -1;
	else fcntl(fd, F_SETFL, 0);
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return 1;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 0;
	config.c_cc[VTIME] = 10;
	cfsetispeed(&config, B57600);
	cfsetospeed(&config, B57600);
	tcsetattr(fd, TCSAFLUSH, &config);
	return fd;
}

int GPSConnection::init_bluetooth(const char *port_name){
	fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)return -1;
	else fcntl(fd, F_SETFL, 0);
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of file %d\n", fd);
		return 1;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 0;
	config.c_cc[VTIME] = 10;
	cfsetispeed(&config, B115200);
	cfsetospeed(&config, B115200);
	tcsetattr(fd, TCSAFLUSH, &config);
	return fd;
}

bool GPSConnection::isOpened(){
	return opened;
}

int GPSConnection::getPortNum(){
	return portNum;
}

// GPSConnection::GPSConnection(QObject *parent)
// 	: QObject(parent) {
//     if(RS232_OpenComport(DEFAULT_PORT_NUM, DEFAULT_BAUD_RATE, "6O2"))
//     {
//         printf("Can not open comport\n");
//     }
// 	isKilled = true;
// }


GPSConnection::~GPSConnection()
{
	close(fd);
}

// void GPSConnection::readGPS()
// {
// 	static int i = 0;
// 	static int j = 0;
// 	static int k = 0;
// 	static int penanda = 0;
//     n = RS232_PollComport(DEFAULT_PORT_NUM, data, 20);

// 	for (int ii = 0; ii<sizeof(data); ii++)
// 	{
// 		if (data[ii] == 'A')
// 		{
// 			penanda = 0;
// 		}
//         else if (data[ii] == 'B'){
// 			penanda = 1;
// 		}
// 		else if (data[ii] == 'C')
// 		{
// 			penanda = 2;
// 		}


// 		switch (penanda)
// 		{
// 		case 0:
// 			if (data[ii] != 'A')
// 			{
// 				lat_kirim[i++] = data[ii];
// 			}
// 			else
// 			{
// 				i = 0;
// 			}
// 			break;
// 		case 1:
// 			if (data[ii] != 'B')
// 			{
// 				long_kirim[j++] = data[ii];
// 			}
// 			else
// 			{
// 				j = 0;
// 			}
// 			break;
// 		case 2:
// 			if (data[ii] != 'C')
// 			{
// 				comp_kirim[k++] = data[ii];

// 			}
// 			else
// 			{
// 				k = 0;
// 			}
// 			break;
// 		}

// 	}

// 	lattDouble = atof(lat_kirim)*(-1);
// 	longDouble = atof(long_kirim);
// 	compassOlah = atof(comp_kirim) / 10.0;

// 	emit serial_kirim_data_GPS(lattDouble, longDouble, compassOlah);
// 	emit sendCurrentPos(lattDouble, longDouble, compassOlah);
// }


// void GPSConnection::sendDatatoSTM()
// {
//    char kirim[40];
//        sprintf(kirim, "AB%4d,%4d,%4d,%4d,%4d,%4d,%4dC", MotorKiri, MotorKanan, ServoKiri, ServoKanan, Servo_Camera, MotorTengah, Servo_Camera2);
//        sprintf(kirim, "AB1500,1500,1500,1500,1500C");
//        serportGPS->write(kirim);
// }
