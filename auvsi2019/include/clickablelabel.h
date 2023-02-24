#ifndef CLICKABLELABEL_H
#define CLICKABLELABEL_H

#include <QLabel>
#include <QWidget>
#include <Qt>
#include <QMouseEvent>


class ClickableLabel : public QLabel {
	Q_OBJECT

public:
	explicit ClickableLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
	~ClickableLabel();

signals:
	void clickedPos(QPoint);
	void initialPos(QPoint);
	void currentPos(QPoint);
	void endPos(QPoint); 

protected:
	//QPoint initPos, curPos;
	void mousePressEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
};

#endif // CLICKABLELABEL_H