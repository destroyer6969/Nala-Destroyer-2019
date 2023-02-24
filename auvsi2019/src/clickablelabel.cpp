#include "clickablelabel.h"
#include <qdebug.h>
ClickableLabel::ClickableLabel(QWidget* parent, Qt::WindowFlags f)
	: QLabel(parent) {

}

ClickableLabel::~ClickableLabel() {}

void ClickableLabel::mousePressEvent(QMouseEvent* event) {
	QPoint point = event->pos();
	qDebug() << point.x() << point.y();
	emit clickedPos(point);
	emit initialPos(point);
}

void ClickableLabel::mouseMoveEvent(QMouseEvent* event) {
	QPoint point = event->pos();
	qDebug() << "Current: "<< point.x() << point.y();
	emit currentPos(point);
}

void ClickableLabel::mouseReleaseEvent(QMouseEvent* event) {
	QPoint point = event->pos();
	qDebug() << "End : " << point.x() << point.y();
	emit endPos(point);
}
