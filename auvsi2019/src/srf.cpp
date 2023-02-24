#include "srf.h"

SRF::SRF(QObject *parent):QObject(parent){
    opened = false;
    running = false;
    portNum = -1;
    for(int q=0;q<5;q++)srfData[q]=-1;
    changingMode = 0;
    baudRate = "115200";
    baudRate_ = "9600";
}
SRF::~SRF(){
    stop();
}

void SRF::run(std::vector<std::string> *portNames){
    if(!opened){
        QMutex m;
        auto it = portNames->begin();
        int a=0;
        char tmp[30], tmp_[30], cmd[30], baud[10];
        FILE *stream;
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
            // if(strcmp(baud,baudRate)!=0 && strcmp(baud,baudRate_)!=0)continue;
            if(init_serial_port(std::string(*it).c_str()) != -1){
                int c=0;
                time(&startTime);
                while((time(NULL)-startTime)<2 && portNum==-1){
                    memset(srfDataStr, 0, sizeof(srfDataStr));
                    result = read(fd, srfDataStr, 25);
                    // qDebug()<<srfDataStr;
                    ++c;
                    if(result && srfDataStr[0]=='A' && srfDataStr[20]=='B'){
                        printf("Port \"%s\" connected to srf\n", std::string(*it).c_str());
                        m.lock();
                        opened = true;
                        portNum = a;
                        m.unlock();
                    }
                    usleep(1);
                }
            }
            if(opened){
                break;
            }
            ++a;
        }
        if(!opened){
            printf("Error: cannot connect to SRF\n");
            m.lock();
            portNum = -2;
            m.unlock();
        }
    }
    int it;
    int i;
    int b;
    int j;
    running = true;
    while(running){
        if(!changingMode){
            memset(srfDataStr, 0, sizeof(srfDataStr));
            result = read(fd, srfDataStr, 25);
            if(result && srfDataStr[0]=='A' && srfDataStr[20]=='B'){
                mtx.lock();
                res = sscanf(srfDataStr, "A%d,%d,%d,%d,%dB", &srfData[0], &srfData[1], &srfData[3], &srfData[2], &srfData[4]);
                if(srfData[4]==-1)srfData[4]=300;
                if(srfData[0]==-1)srfData[0]=300;
                if(srfData[1]==-1)srfData[1]=300;
                if(srfData[2]==-1)srfData[2]=300;
                if(srfData[3]==-1)srfData[3]=300;
                // qDebug()<<srfDataStr;
                // qDebug()<<srfData[0]<<", "<<srfData[1]<<", "<<srfData[2]<<", "<<srfData[3]<<", "<<srfData[4];
                mtx.unlock();
                emit sendData(srfData[0], srfData[1], srfData[2], srfData[3], srfData[4]);
            }
        }
        usleep(100);
    }
}

void SRF::stop(){
	if(close(fd))qDebug()<<"Error Closing SRF Port";
	else{
		qDebug()<<"SRF disconnected";
	}
    mtx1.lock();
    memset(srfData, -1, sizeof(srfData));
    running = false;
	opened = false;
    portNum = -1;
    mtx1.unlock();
}

int SRF::init_serial_port(const char *port_name){
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
	cfsetispeed(&config, B115200);
	cfsetospeed(&config, B115200);
	tcsetattr(fd, TCSAFLUSH, &config);
	return fd;
}

int* SRF::getSRFData(){
    return srfData;
}

bool SRF::isOpened(){
    return opened;
}

int SRF::getPortNum(){
    return portNum;
}

void SRF::setSRFMode(std::string &str){
    QMutex a;
    a.lock();
    changingMode = 1;
    a.unlock();
    for(int q=0;q<20;q++){
        write(fd, str.c_str(), str.size());
        usleep(1000);
    }
    a.lock();
    changingMode = 0;
    a.unlock();
}
