#include "labelmapping.h"
#include <qdebug.h>

LabelMapping::LabelMapping(QWidget *parent)
	: QOpenGLWidget(parent)
{
    map_ = cv::Mat(860,960, CV_8UC3, cvScalar(4, 12, 178));
    for(int i=20;i<960;i+=20)cv::line(map_, cv::Point(i,0), cv::Point(i, 859), cv::Scalar(100,100,100), 1);
    for(int i=20;i<860;i+=20)cv::line(map_, cv::Point(0,i), cv::Point(959, i), cv::Scalar(100,100,100), 1);
    map = QPixmap::fromImage(QImage((unsigned char*) map_.data, map_.cols, map_.rows, QImage::Format_RGB888));
    gmap = QPixmap(":images/danau8.jpg").scaled(QSize(960, 860), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    outMap = &map;
    gmapReady = false;
    // this->setMouseTracking(true);
    initCircle = QPoint(0,0);
    startPos = QPoint(-1,-1);
    nextCircle.misi = 1;
    nextCircle.index= 0;
    resetTransformation();
    centerMap.setX(480);
    centerMap.setY(430);
    topLeftCorner.setX(-480);
    topLeftCorner.setY(-430);
    clearCircles();
    boat = new QImage(":images/small_boat.png");
    boatHalfWidth = boat->width()/2;
    boatHalfHeight = boat->height()/2;
    yellow2 = QPen(Qt::yellow, 2); red2 = QPen(Qt::red, 2); labelPen = QPen(QColor(255, 118, 20)); circlePen = QPen(Qt::black);
    t = new QTimer(0);
    connect(t, SIGNAL(timeout()), this, SLOT(updateAll()), Qt::QueuedConnection);
    t->start(50);
    qDebug() << "Label mapping has been created";
    dronePos.setX(0);dronePos.setY(0);
}

LabelMapping::~LabelMapping()
{
    t->stop();
    delete t;
    delete gmapPainter;
    delete boat;
}

void LabelMapping::mouseMoveEvent(QMouseEvent *event)
{
    QPoint pos = event->pos();
    emit sendMousePosition(pos);
}

void LabelMapping::mousePressEvent(QMouseEvent *event)
{
    QPoint pt = event->pos();


    if (event->button() == Qt::LeftButton)
            emit leftClicked(pt);
    else if (event->button() == Qt::RightButton)
            emit rightClicked(pt);
}

void LabelMapping::mouseDoubleClickEvent(QMouseEvent *event){
    QPoint pt = event->pos();
    if(event->button() == Qt::LeftButton)
        emit doubleClicked(pt);
}

void LabelMapping::paintEvent(QPaintEvent *){
    QPainter painter(this);

    painter.translate(centerMap);
    painter.rotate(mapRotationDegree);
    painter.translate(topLeftCorner+mapTranslationValue);
    painter.drawPixmap(0,0,*outMap);

//    draw line
    if(!circles[nextCircle.misi].empty() && NUM_CIRCLES[nextCircle.misi] > nextCircle.index){
        painter.setPen(yellow2);
        painter.drawLine(initCircle, circles[nextCircle.misi][nextCircle.index].pt);
    }

//    draw control line
    if(startPos.x()!=-1 && !circles[nextCircle.misi].empty() && NUM_CIRCLES[nextCircle.misi] > nextCircle.index){
        painter.setPen(red2);
        painter.drawLine(startPos, circles[nextCircle.misi][nextCircle.index].pt);
    }

//    draw waypoints as circles
    painter.setPen(circlePen);
    for(unsigned i=1; i<NUM_CIRCLES_CAT; i++){
        if(circles[i].empty()){
            continue;
        }
        for(unsigned iter=0; iter<NUM_CIRCLES[i]; iter++){
            if(circles[i][iter].pt == NULLPOINT) continue;
            else if(circles[i][iter].visited == 1){
                painter.setBrush(Qt::gray);
                painter.drawEllipse(circles[i][iter].pt, 4, 4);
            }
            else{
                painter.setBrush( i==8 ? Qt::red : Qt::green);
                painter.drawEllipse(circles[i][iter].pt, 5, 5);
            }
        }
    }

    if(enableLabel){
        painter.setPen(labelPen);
        for (int i=1; i<NUM_CIRCLES_CAT; i++) {
            if(circles[i].empty()) continue;
            for(unsigned iter=0; iter<NUM_CIRCLES[i]; iter++){
                if(circles[i][iter].pt == NULLPOINT) continue;
                painter.drawText(circles[i][iter].pt, QString(circles[i][iter].labeltext));
            }
        }
    }

//    draw drone's position
    /*
    if(dronePos != NULLPOINT){
        painter.setBrush(Qt::red);
        painter.drawEllipse(dronePos, 5, 5);

    }
    */
//    draw current position as initCircle
    painter.translate(initCircle);
    painter.rotate(compass);
    painter.drawImage(-boatHalfWidth,-boatHalfHeight, *boat);
    if(enablePath)gmapPainter->drawPoint(initCircle);
}

void LabelMapping::changeInitCircle(const QPoint& pt){
    initCircle = pt;
}

void LabelMapping::addCircle(unsigned misiIndex, unsigned index, MapPoint pt){
    if (index >= NUM_CIRCLES[misiIndex]){
        circles[misiIndex].push_back(pt);
        NUM_CIRCLES[misiIndex]++;
    }
    else
        circles[misiIndex][index] = pt;
}

void LabelMapping::clearCircles(){
    for(int index=0; index<NUM_CIRCLES_CAT; index++){
        circles[index].clear();
        NUM_CIRCLES[index] = 0;
    }
}

void LabelMapping::deleteCircle(int misiIdx, int wpIdx){
    if(NUM_CIRCLES[misiIdx] && wpIdx<NUM_CIRCLES[misiIdx]){
        circles[misiIdx].erase(circles[misiIdx].begin()+wpIdx);
        NUM_CIRCLES[misiIdx]--;
    }
}

QPoint& LabelMapping::getInitCircle(){
    return this->initCircle;
}

void LabelMapping::toggleLabel(int state){
    this->enableLabel = state;
}

void LabelMapping::togglePath(int state){
    this->enablePath = state;
}

void LabelMapping::setNextCircle(int wpIdx){
    if(nextCircle.index!=wpIdx && nextCircle.index<circles[nextCircle.misi].size())circles[nextCircle.misi][nextCircle.index].visited = 1;
    nextCircle.index = wpIdx;
}

void LabelMapping::setNextMisi(int misiIdx){
    nextCircle.misi = misiIdx;
    nextCircle.index = 0;
}

void LabelMapping::unvisitAllWaypoints(){
    for(int i=1;i<NUM_CIRCLES_CAT;i++){
        for(int j=0;j<NUM_CIRCLES[i];j++){
            circles[i][j].visited = 0;
        }
    }
}

void LabelMapping::setMapImage(QPixmap *map_){
    // this->map = map_;
}

void LabelMapping::getNewAngle(double compass){
    this->compass = compass;
}
void LabelMapping::rotateMapRight(){
    this->mapRotationDegree += 5;
    this->mapRotationRadian = this->mapRotationDegree*PI/180.0;
}

void LabelMapping::rotateMapLeft(){
    this->mapRotationDegree -= 5;
    this->mapRotationRadian = this->mapRotationDegree*PI/180.0;
}

void LabelMapping::translateMapUp(){
    this->mapTranslationValue.setY(mapTranslationValue.y()-5);
}

void LabelMapping::translateMapRight(){
    this->mapTranslationValue.setX(mapTranslationValue.x()+5);
}

void LabelMapping::translateMapDown(){
    this->mapTranslationValue.setY(mapTranslationValue.y()+5);
}

void LabelMapping::translateMapLeft(){
    this->mapTranslationValue.setX(mapTranslationValue.x()-5);
}

void LabelMapping::normalizePosition(QPoint &pt){
    float cosAngle = cos(-this->mapRotationRadian);
    float sinAngle = sin(-this->mapRotationRadian);
    float x = pt.x()-this->centerMap.x();
    float y = pt.y()-this->centerMap.y();
    pt.setX((x*cosAngle - y*sinAngle) + this->centerMap.x());
    pt.setY((x*sinAngle + y*cosAngle) + this->centerMap.y());
    pt.setX(pt.x()-mapTranslationValue.x());
    pt.setY(pt.y()-mapTranslationValue.y());
}

void LabelMapping::resetTransformation(){
    this->mapRotationDegree = 0;
    this->mapRotationRadian = 0;
    this->mapTranslationValue.setX(0);
    this->mapTranslationValue.setY(0);
}

void LabelMapping::sendingImage(){
    emit sendImage(&map);
}

void LabelMapping::getGMap(const char *map_path_){
    if(gmapReady){
        QMutex q;
        q.lock();
        gmapPainter->end();
        delete gmapPainter;
        q.unlock();
    }
    gmapOri = QPixmap(map_path_).scaled(QSize(960, 860), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    gmap = gmapOri;
    gmapPainter = new QPainter(&gmap);
    gmapPainter->setPen(QColor(64, 64, 64));
    for(int i=20;i<960;i+=20)gmapPainter->drawLine(QPoint(i, 0), QPoint(i, 859));
    for(int i=20;i<860;i+=20)gmapPainter->drawLine(QPoint(0, i), QPoint(959, i));
    gmapPainter->setPen(QColor(0, 255, 255));
    gmapReady = true;
}

void LabelMapping::mapMode(int state){
    if(state) outMap = &gmap;
    else outMap = &map;
}

void LabelMapping::resetMap(){
    QMutex a;
    a.lock();
    gmapPainter->end();
    gmap = gmapOri;
    gmapPainter->begin(&gmap);
    gmapPainter->setPen(QColor(64, 64, 64));
    for(int i=20;i<960;i+=20)gmapPainter->drawLine(QPoint(i, 0), QPoint(i, 859));
    for(int i=20;i<860;i+=20)gmapPainter->drawLine(QPoint(0, i), QPoint(959, i));
    gmapPainter->setPen(QColor(0, 255, 255));
    a.unlock();
}

void LabelMapping::updateAll(){
    update();
    updateAttitude();
}

void LabelMapping::setStartPos(int y, int x){
    startPos.setX(x);
    startPos.setY(y);
}


void LabelMapping::changeDronePos(const QPoint& pt){
    dronePos = pt;
}
