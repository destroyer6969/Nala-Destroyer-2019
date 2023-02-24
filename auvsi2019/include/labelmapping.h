#pragma once
#include <qobject.h>
#include <qwidget.h>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QTimer>
#include <QThread>
#include <QOpenGLWidget>
#include <vector>
#include <qdebug.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <QMutex>

using namespace std;

#define NULLPOINT           QPoint(-1,-1)
#define NUM_CIRCLES_CAT     11              // = jumlah misi
#define PI 3.14159265

typedef struct{
    QPoint pt;
    QString labeltext;
    bool visited = 0;
} MapPoint;

class LabelMapping : public QOpenGLWidget
{
	Q_OBJECT

public:
        LabelMapping(QWidget *parent = NULL);
        ~LabelMapping();
        void deleteCircle(int, int);
        QPoint& getInitCircle();
        void setNextCircle(int);
        void restartWaypoints(int wpIdx, int misiIdx);
        void normalizePosition(QPoint&);
        void setStartPos(int, int);

private:
        int enableLabel, enablePath;
        int mapRotationDegree;
        bool gmapReady;
        float mapRotationRadian;
        QPoint mapTranslationValue;
        QPoint centerMap;
        QPoint topLeftCorner;
        QPixmap map, gmap, gmapOri;
        QPixmap *outMap;
        QTimer *t;
        QThread *timerThread;
        QPoint initCircle, startPos, dronePos;
        QMutex mtx;
        QPainter *gmapPainter;
        QPen yellow2, red2, labelPen, circlePen;
        cv::Mat map_;
        double compass;
        unsigned NUM_CIRCLES[NUM_CIRCLES_CAT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        vector<MapPoint> circles[NUM_CIRCLES_CAT];
        struct{
            unsigned misi;
            unsigned index;
        } nextCircle;
        QImage *boat;
        int boatHalfWidth, boatHalfHeight;
        void mouseMoveEvent(QMouseEvent *event);
        void mousePressEvent(QMouseEvent *event);
        void mouseDoubleClickEvent(QMouseEvent *event);
        void paintEvent(QPaintEvent *event);

signals:
	void sendMousePosition(QPoint&);
        void leftClicked(QPoint&);
        void rightClicked(QPoint&);
        void doubleClicked(QPoint&);
        void sendImage(QPixmap*);
        void updateAttitude();

public slots:
        void setNextMisi(int);
        void setMapImage(QPixmap*);
        void getNewAngle(double);
        void rotateMapRight();
        void rotateMapLeft();
        void translateMapUp();
        void translateMapRight();
        void translateMapDown();
        void translateMapLeft();
        void resetTransformation();
        void sendingImage();
        void getGMap(const char*);
        void mapMode(int);
        void unvisitAllWaypoints();
        void resetMap();
        void toggleLabel(int);
        void togglePath(int);

private slots:
        void changeInitCircle(const QPoint&);
        void changeDronePos(const QPoint&);
        void addCircle(unsigned, unsigned, MapPoint);
        void clearCircles();
        void updateAll();
};
