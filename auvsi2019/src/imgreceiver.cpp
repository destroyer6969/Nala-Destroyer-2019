#include "imgreceiver.h"
#include <QDebug>
#include <QMutex>
#include <std_msgs/Bool.h>

ImgReceiver::ImgReceiver(QObject *parent)
    : QObject(parent),
      itrans(n)
{
    isKilled = false;

    //initialize hsv erode dilate
//    memset(hmin, 0, NUM_COLORS);
//    memset(smin, 0, NUM_COLORS);
//    memset(vmin, 0, NUM_COLORS);
//    memset(hmax, 255, NUM_COLORS);
//    memset(smax, 255, NUM_COLORS);
//    memset(vmax, 255, NUM_COLORS);
//    memset(erode, 0, NUM_COLORS);
//    memset(dilate, 0, NUM_COLORS);
    isThreshold = 0;
    for(int i=0; i<NUM_COLORS; i++){
        hmin[i]=0;
        smin[i]=0;
        vmin[i]=0;
        hmax[i]=0;
        smin[i]=0;
        vmin[i]=0;
        erode[i]=0;
        dilate[i]=0;
    }

    currentColor = 0;

    camSwitch_pub = n.advertise<std_msgs::Int8>("cam_switch", 1);
    camToSwitch = -1;
}

void ImgReceiver::kill(){
    isKilled = true;
    qDebug() << "ImgReceiver killed";
    ros::shutdown();
}

void ImgReceiver::start(){
    isKilled = false;
    if(isThreshold){
        cap = VideoCapture(0);
        if(!cap.isOpened()){
            qDebug() << "Could not open camera";
        }
        while(isKilled==false){
            cap >> frame;
            processImage(frame);

            // clear msg
            for(int i=0; i<10; i++)
                msg.detected_object[i].boxes.clear();
        }
        emit imgReceived(matToQImage(Mat()));
        emit sendBinaryImage(matToQImage(Mat()));
        if(cap.isOpened())
            cap.release();
    }

    else{
        do{
            if(isKilled){
                return;
            }
            // qDebug() <<"looking for publisher";
            objAttr_sub = n.subscribe("object_attr", 1, &ImgReceiver::objAttrReceive, this);
        } while(!isKilled && objAttr_sub.getNumPublishers()==0);

        qDebug() << "Got publisher";
        while(isKilled==false){
            if(camToSwitch>=0){
                camSwitch_msg.data = camToSwitch;
                camSwitch_pub.publish(camSwitch_msg);
                camToSwitch = -1;
            }
            ros::spinOnce();
        }

        img_sub.shutdown();
    }
}


void ImgReceiver::stop(){
    isKilled = true;
}

ImgReceiver::~ImgReceiver(){
    qDebug() << "ImgReceiver destroyed";
}

void ImgReceiver::switchCam(int cam_idx){
    mtx_cam.lock();
    camToSwitch = cam_idx;
    mtx_cam.unlock();
    qDebug()<< "Switching camera to: " << cam_idx;
}

void ImgReceiver::objAttrReceive(const object_detection::ObjectAttr &msg){
    if(isThreshold) return;
    cv_bridge::CvImagePtr cvPtr;
    try{
        cvPtr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_INFO("cv_bridge exception %s", e.what());
        return;
    }

    emit imgReceived(matToQImage(cvPtr->image));
    emit sendDetectedObject(msg);

    // qDebug() << msg.xmin << " " << msg.ymin << " " << msg.xmax << " " << msg.ymax << " " << msg.score;
}

void ImgReceiver::processImage(Mat &img){
    thresholdImage(img);
    cvtColor(frame, frame, COLOR_BGR2RGB);
    emit imgReceived(matToQImage(frame));
    emit sendDetectedObject(this->msg);
}


QImage* ImgReceiver::matToQImage(const cv::Mat& mat){
    // qDebug() << "converting image";
    if(mat.type() == CV_8UC1)
        return new QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);       // return QImage Grayscale
    else if(mat.type() == CV_8UC3)
        return new QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);       // return QImage
    return new QImage();                                         // return a blank QImage if the above did not work
}

void ImgReceiver::thresholdImage(cv::Mat &img){
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    for (int idxColor=0; idxColor<NUM_COLORS; idxColor++){
        detectObjectByColor(&hsv, idxColor);
    }
}

void ImgReceiver::detectObjectByColor(cv::Mat *hsv, int idxColor){
    Mat bin;
    inRange(*hsv, Scalar(hmin[idxColor], smin[idxColor], vmin[idxColor]),
            Scalar(hmax[idxColor], smax[idxColor], vmax[idxColor]), bin);
    morphOps(&bin, idxColor);

    // find contours
    object_detection::BoundingBox box;
    cv::Mat temp;
    bin.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    cv::Rect rect;
    if (hierarchy.size() > 0) {
            int numObjects = hierarchy.size();
            //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
            if (numObjects<MAX_NUM_OBJECTS) {
                    for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                            cv::Moments moment = moments((cv::Mat)contours[index]);
                            double area = moment.m00;
                            //if the area is less than 20 px by 20px then it is probably just noise
                            //if the area is the same as the 3/2 of the image size, probably just a bad filter
                            //we only want the object with the largest area so we safe a reference area each
                            //iteration and compare it to the area in the next iteration.
                            if ((area>MIN_OBJECT_AREA)&(area<MAX_OBJECT_AREA)) {
                                rect = cv::boundingRect(contours[index]);
                                box.xmin = rect.x;
                                box.ymin = rect.y; // x first y second
                                box.xmax = rect.x + rect.width;
                                box.ymax = rect.y + rect.height;

                                // classify object
                                if(2 * rect.width <= rect.height ){
                                    // CAN_BUOY
                                    if(idxColor == RED){
                                        msg.detected_object[RED_CAN_BUOY].boxes.push_back(box);
                                    }
                                    else if(idxColor == GREEN){
                                        msg.detected_object[GREEN_CAN_BUOY].boxes.push_back(box);
                                    }
                                    else{
                                        msg.detected_object[ANOTHER_CAN_BUOY].boxes.push_back(box);
                                    }
                                }
                                else{
                                    // BALL
                                    msg.detected_object[BALL].boxes.push_back(box);
                                }
                            }
                    }
                    // draw rects bounding detected objects
                    drawRectangles();
            }
            else putText(frame, "TOO MUCH NOISE! ADJUST FILTER", cv::Point(0, 50), 1, 2, cv::Scalar(0, 0, 255), 2);
    }


    if(idxColor==currentColor){
       sendBinaryImage(matToQImage(bin));
    }
}

void ImgReceiver::morphOps(cv::Mat *bin, int idxColor){
    if(erode[idxColor]>0)
        cv::erode(*bin, *bin, getStructuringElement(MORPH_RECT,Size(erode[idxColor], erode[idxColor])));
    if(dilate[idxColor]>0)
        cv::dilate(*bin, *bin, getStructuringElement(MORPH_RECT,Size(dilate[idxColor], dilate[idxColor])));
    if(dilate[idxColor]>0)
        cv::erode(*bin, *bin, getStructuringElement(MORPH_RECT,Size(dilate[idxColor], dilate[idxColor])));
    if(erode[idxColor]>0)
        cv::erode(*bin, *bin, getStructuringElement(MORPH_RECT,Size(erode[idxColor], erode[idxColor])));
}


void ImgReceiver::colorChange(int idx){
    this->currentColor=idx;
}

void ImgReceiver::hminChange(int x){
    hmin[currentColor] = x;
}

void ImgReceiver::sminChange(int x){
    smin[currentColor] = x;
}

void ImgReceiver::vminChange(int x){
    vmin[currentColor] = x;
}

void ImgReceiver::hmaxChange(int x){
    hmax[currentColor] = x;
}

void ImgReceiver::smaxChange(int x){
    smax[currentColor] = x;
}

void ImgReceiver::vmaxChange(int x){
    vmax[currentColor] = x;
}

void ImgReceiver::erodeChange(int x){
    erode[currentColor] = x;
}

void ImgReceiver::dilateChange(int x){
    dilate[currentColor] = x;
}

void ImgReceiver::drawRectangles(){
    object_detection::BoundingBox box;
    for(int i=0; i<10; i++){
        for(int idx=0; idx<msg.detected_object.data()[i].boxes.size(); idx++){
            box = msg.detected_object.data()[i].boxes.data()[idx];
            rectangle(frame, cvPoint(box.xmin, box.ymin), cvPoint(box.xmax, box.ymax), Scalar(0,255,0), 5);
        }
    }
}





















