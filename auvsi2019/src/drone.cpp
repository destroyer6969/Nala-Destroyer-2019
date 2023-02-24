#include "drone.h"

Drone::Drone(QObject *parent): QObject(parent){
	companion_id = 0;
	opened = 0;

	travelingstate=0;
	gotNedPos = 0;
	gotGlobalPos = 0;
}
Drone::~Drone(){
    disconnect();
}

bool Drone::isOpened(){
	return opened;
}

void Drone::initPixhawkConnection(std::vector<std::string>*portNames){
    if(!opened){
		QMutex m;
		auto it=portNames->begin();
		int a=0;
        char tmp[30], tmp_[30], cmd[30], baud[10];
        FILE *stream;
		portNum = -1;
		for(it; it!=portNames->end(); it++){
            memset(baud, 0, sizeof(baud));
            QMutex mm1;
            mm1.lock();
			memset(tmp, 0, sizeof(tmp));
			memset(cmd, 0, sizeof(cmd));
            strcpy(tmp, std::string(*it).c_str());
            strcpy(cmd, "stty -F ");
            strcat(cmd, std::string(*it).c_str());
            mm1.unlock();
            tmp[strlen(tmp)-1]=0;
            if(strcmp(tmp, "/dev/ttyUSB")!=0)continue;
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
			qDebug()<<std::string(*it).c_str();
            // qDebug()<<baud<<"test";
            // if(strcmp(baud,baudRate)!=0)continue;
			if(init_serial_port("/dev/ttyUSB1") != -1){
				time(&startTime);
				while((time(NULL)-startTime)<3 && portNum==-1){
					result = read(fd, &cp, 1);
					if(result>0) mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
					if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT){
						mavlink_msg_heartbeat_decode(&message, &heartBeat);
						if(heartBeat.type == 2){
							system_id = message.sysid;
							qDebug()<<message.sysid;
							autopilot_id = message.compid;
							printf("Port \"%s\" connected to drone's Pixhawk\n", std::string(*it).c_str());
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
			printf("Error: cannot connect to drone's pixhawk\n");
			m.lock();
			portNum = -2;
			m.unlock();
		}
	}
}

int Drone::init_serial_port(const char *port_name){
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

void Drone::toggle_offboard_control( bool flag ){
	mavlink_message_t message_;
	uint8_t mode = flag ? 4 : 5;
	mavlink_msg_set_mode_pack(system_id, 0, &message_, 0, 1,  4);
	sendMessage(message_);
}

void Drone::set_auto(bool flag){
	mavlink_command_long_t cmd = { 0 };
	cmd.target_system    = system_id;
	cmd.target_component = autopilot_id;
	cmd.command          = MAV_MODE_FLAG_AUTO_ENABLED;
	cmd.confirmation     = true;
	cmd.param1           = (float) flag;
	mavlink_message_t message_;
	mavlink_msg_command_long_encode(system_id, companion_id, &message_, &cmd);
	sendMessage(message_);
}

void Drone::sendMessage(const mavlink_message_t &message){
	char buf[300];
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	qDebug()<<"sending command";
    write(fd, buf, len);
	tcdrain(fd);
}

void Drone::disconnect(){
	toggle_offboard_control(0);
	usleep(100000);
	arm_disarm(0);
	usleep(100000);
	QMutex mtx1;
	mtx1.lock();
	if(close(fd))qDebug()<<"Error Closing Drone's Pixhawk Port";
	else{
		qDebug()<<"Drone's Pixhawk disconnected";
	}
	opened = 0;
	mtx1.unlock();
}

void Drone::readMessage(){
	result = read(fd, &cp, 1);
	if(result>0) mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
	if(message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED){
		mavlink_msg_local_position_ned_decode(&message, &nedPos);
		gotNedPos = 1;
		// timestamp = nedPos.time_boot_ms;
	}
	else if(message.msgid == MAVLINK_MSG_ID_ATTITUDE){
		mavlink_msg_attitude_decode(&message, &attitude);
		timestamp = attitude.time_boot_ms;
	}
	else if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT){
		mavlink_msg_heartbeat_decode(&message, &heartBeat);
		// printf("Heartbeat\n");
	}
	else if(message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT){
		mavlink_msg_global_position_int_decode(&message, &global_pos);
		tmpLat = (double)global_pos.lat/(double)10000000;
		tmpLon = (double)global_pos.lon/(double)10000000;
		tmpCompass = (double)global_pos.hdg/(double)100;
		if(tmpLat==0)return;
		if(tmpLon==0)return;
		if(tmpCompass==0)return;
		latitude = tmpLat;
		longitude = tmpLon;
		compass = tmpCompass;
		gotGlobalPos = 1;
		// timestamp = global_pos.time_boot_ms;
		// printf("%lf, %lf\n", latitude, longitude);
		// qDebug()<<"lat: "<<latitude<<", lon: " <<longitude;
		emit sendPos(latitude, longitude);
	}
	else if(message.msgid == MAVLINK_MSG_ID_COMMAND_ACK){
		mavlink_msg_command_ack_decode(&message, &ack);
		qDebug()<<"command id: "<<ack.command;
		qDebug()<<"result: "<<ack.result<<"\n";
	}
}

void Drone::change_speed(int type, float speed, int throttle){
	mavlink_command_long_t cmd;
	mavlink_message_t msg;
	char buf[300];
	cmd.target_system = system_id;
	cmd.target_component = 0;
	cmd.command = MAV_CMD_DO_CHANGE_SPEED;
	cmd.param1 = type;
	cmd.param2 = speed;
	cmd.param3 = throttle;
	cmd.param4 = 0;
	mavlink_msg_command_long_encode(system_id, 0, &msg, &cmd);
	sendMessage(msg);
}

bool Drone::isArrived(int idx){
	double destLatitude, destLongitude; 
	if(idx>=0){
		destLatitude = pos[nextPos_idx].lat;
		destLongitude = pos[nextPos_idx].lon;
	}
	if(idx == -2){
		destLatitude = launchLatitude;
		destLongitude = launchLongitude;
	}
	else{
		destLatitude = homeLatitude;
		destLongitude = homeLongitude;
	}
	jarak       = sqrtf64(
                pow(destLatitude - latitude, 2)
                + pow(destLongitude - longitude, 2)
                );
	if(idx==-1){
		if(fabs(latitude -  homeLatitude) < MinRadius_lat &&
				fabs(longitude - homeLongitude) < MinRadius_lon )return true;
	}
	else{
		if( fabs(latitude -  pos[nextPos_idx].lat) < MinRadius_lat &&
			fabs(longitude - pos[nextPos_idx].lon) < MinRadius_lon ){
			printf("pos-dest: %lf-%lf, %lf-%lf\n", latitude, destLatitude, longitude, destLongitude);
			// printf("distance: %lf, %lf\n", fabs(latitude -  destLatitude), fabs(longitude - destLongitude));
			return true;
		}
	}
	return false;
}

void Drone::launch(){
	// mtx.lock();
	travelingstate = -1;
	initCondition();
	toggle_offboard_control(0);
	usleep(100000);
	toggle_offboard_control(1);
	usleep(100000);
	arm_disarm(1);
	usleep(100000);
	change_speed(0, 0.5);
	usleep(100000);
	change_speed(1, 0.5);
	usleep(100000);
	change_speed(2, 3);
	usleep(100000);
	change_speed(3, 0.5);
	usleep(100000);
	takeoff(5);
	sleep(3);
	// mtx.unlock();
	qDebug()<<"launch";
	start_traveling();
	// if(travelingstate){
	// 	if(rtl)
	// 		returnToLaunch();
	// 	else
	// 		return_to_home();
	// }
	usleep(1000000);
	setRTL();
}

void Drone::start_traveling(){
	nextPos_idx = 0;
	rtl = 0;
	travelingstate = 1;
	while(travelingstate==1 && !rtl && nextPos_idx < pos.size()){
		printf("go to : %lf %lf\n", pos[nextPos_idx].lat, pos[nextPos_idx].lon);
		goto_pos(pos[nextPos_idx].lat, pos[nextPos_idx].lon, pos[nextPos_idx].alt);
		do {
			readMessage();
			// printf("pos-dest: %lf-%lf, %lf-%lf\n", latitude, pos[nextPos_idx].lat, longitude, pos[nextPos_idx].lon);
			// printf("distance: %lf, %lf\n", fabs(latitude -  pos[nextPos_idx].lat), fabs(longitude - pos[nextPos_idx].lon));
			usleep(100);
		} while(!isArrived(nextPos_idx) && travelingstate==1);
		++nextPos_idx;
		qDebug()<<nextPos_idx;
	}
	usleep(5000000);
	if (travelingstate==1)
		travelingstate = 2;
}

void Drone::setRTL(){
	// mtx.lock();
	// rtl = 1;
	// homeLatitude = launchLatitude;
	// homeLongitude = launchLongitude;
	// qDebug()<<launchLatitude << " " << launchLongitude;
	// mtx.unlock();
	qDebug() << "set rtl";
	if(homeLatitude==0 && homeLongitude==0){
		// returnToLaunch();
	}
	else return_to_home();
}

void Drone::returnToLaunch(){
	qDebug()<<"return to launch";
	mavlink_command_long_t cmd;
	mavlink_message_t msg;
	cmd.target_system = system_id;
	cmd.target_component = 0;
	cmd.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
	mavlink_msg_command_long_encode(system_id, 0, &msg, &cmd);
	sendMessage(msg);
	travelingstate = 2;
}

void Drone::setTraveling(int state){
	QMutex mmm;
	mmm.lock();
	travelingstate = state;
	mmm.unlock();
}

void Drone::goto_pos(float lat, float lon, float alt, float yaw, float vx_, float vy_, float vz_){
	mavlink_set_position_target_global_int_t cmd;
	mavlink_message_t msg;
	cmd.target_system = system_id;
	cmd.target_component = 0;
	cmd.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	cmd.type_mask = 0b0000111111111000; // ignore velocity, acceleration, yaw, yaw rate
	cmd.lat_int = (int)(lat*10000000.0);
	cmd.lon_int = (int)(lon*10000000.0);
	cmd.alt = alt;
	mavlink_msg_set_position_target_global_int_encode(system_id, 0, &msg, &cmd);
	sendMessage(msg);
}

void Drone::takeoff(float alt, float yaw){
    mavlink_command_long_t cmd;
	mavlink_message_t msg;
	cmd.target_system = system_id;
	cmd.target_component = 0;
	cmd.command = MAV_CMD_NAV_TAKEOFF;
	cmd.param1 = cmd.param5 = cmd.param6 = 0;
	cmd.param4 = yaw;
	cmd.param7 = alt;
	mavlink_msg_command_long_encode(system_id, 0, &msg, &cmd);
	sendMessage(msg);
	travelingstate = 3;
	double ZZ = nedPos.z;
	usleep(5000000);
	launchLatitude = latitude;
	launchLongitude = longitude;
	travelingstate = 0;
	qDebug()<<"selesai takeoff";
}

void Drone::land(){
	qDebug() << "start landing";
	mavlink_command_long_t cmd;
	mavlink_message_t msg;
	cmd.target_system = system_id;
	cmd.target_component = 0;
	cmd.command = MAV_CMD_NAV_LAND;
	cmd.param1 = 0;
	cmd.param2 = 0;
	cmd.param4 = NAN;
	cmd.param5 = cmd.param6 = 0;
	cmd.param7 = 0;
	mavlink_msg_command_long_encode(system_id, 0, &msg, &cmd);
	sendMessage(msg);
	travelingstate=4;
	while(travelingstate==4 && nedPos.z < -0.5){
		readMessage();
		usleep(100);
	}
	qDebug()<<"Selesai landing";
	travelingstate = 0;
}

void Drone::arm_disarm(bool arm){
    mavlink_command_long_t cmd;
	mavlink_message_t msg;
	char buf[300];
	cmd.target_system = system_id;
	cmd.target_component = 0;
	cmd.command = 400;
	cmd.param1 = arm;
	mavlink_msg_command_long_encode(system_id, 0, &msg, &cmd);
	sendMessage(msg);
}

void Drone::checkArm(){
	toggle_offboard_control(0);
	usleep(100000);
	toggle_offboard_control(1);
	usleep(100000);
	arm_disarm(1);
	usleep(3000000);
	arm_disarm(1);
	usleep(100000);
	// launch();
	// takeoff(5);
	toggle_offboard_control(0);
	usleep(100000);
}

void Drone::return_to_home(){
	qDebug()<<"return to home";
	goto_pos(homeLatitude, homeLongitude, 5.0);
	usleep(100000);
	travelingstate = 2;
	while(travelingstate==2 && !isArrived(-1)){
		readMessage();
		usleep(100);
	}
	land();
	travelingstate = 0;
}
void Drone::mission_start(int first, int last){
	mavlink_command_long_t cmd = { 0 };
	cmd.target_system    = system_id;
	cmd.target_component = 0;
	cmd.command          = MAV_CMD_MISSION_START;
	cmd.param1           = first;
	cmd.param2			 = last;
	mavlink_message_t message_;
	mavlink_msg_command_long_encode(system_id, companion_id, &message_, &cmd);
	sendMessage(message_);
}

void Drone::setWaypoint(unsigned index, double latitude, double longitude){
	QMutex mmtx;
	mmtx.lock();
	if (index < pos.size()){
		pos[index].lat = latitude;
		pos[index].lon = longitude;
	}
	else {
		Pos_t newpos = {
			.lat = latitude,
			.lon = longitude,
                        .alt = 5,
		};
		pos.push_back(newpos);
	}
	mmtx.unlock();
	// qDebug() << index << " " << latitude << " " << longitude << pos.size();
}

void Drone::deleteWaypoint(unsigned index){
	if(index <= pos.size())
		pos.erase(pos.begin() + index);
}

void Drone::getHomeLatitude(const QString &lat_){
	const char *buf = lat_.toLocal8Bit().data();
	homeLatitude = atof(buf);
	qDebug()<<homeLatitude;
}

void Drone::getHomeLongitude(const QString &lon_){
	const char *buf = lon_.toLocal8Bit().data();
	homeLongitude = atof(buf);
	qDebug()<<homeLongitude;
}

int Drone::getTravelingState(){
	return travelingstate;
}

void Drone::initCondition(){
	gotNedPos = gotGlobalPos = 0;
	while(gotNedPos==0 || gotGlobalPos==0){
		readMessage();
	}
}
