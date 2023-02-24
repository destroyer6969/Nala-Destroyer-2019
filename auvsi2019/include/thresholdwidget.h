#ifndef THRESHOLDWIDGET_H
#define THRESHOLDWIDGET_H

#include <QObject>
#include <QWidget>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QJsonObject>

class ThresholdWidget : public QGroupBox
{
    Q_OBJECT
public:
  explicit ThresholdWidget(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
  void initConnection();
  void write(QJsonObject& obj) const;   // write attributes to JSON to be saved
  void load(const QJsonObject &obj);    // load attributes from JSON

  QWidget *horizontalLayoutWidget;
  QHBoxLayout *horizontalLayout;
  QFormLayout *formLayout;
  QLabel *label;
  QSlider *hminSlider;
  QSpinBox *hminLabel;
  QLabel *label_3;
  QSlider *vminSlider;
  QSpinBox *vminLabel;
  QLabel *label_5;
  QSlider *sminSlider;
  QSpinBox *sminLabel;
  QLabel *label_7;
  QSlider *erodeSlider;
  QSpinBox *erodeLabel;
  QFormLayout *formLayout_2;
  QLabel *label_9;
  QSlider *hmaxSlider;
  QSpinBox *hmaxLabel;
  QLabel *label_11;
  QSlider *vmaxSlider;
  QSpinBox *vmaxLabel;
  QLabel *label_13;
  QSlider *smaxSlider;
  QSpinBox *smaxLabel;
  QLabel *label_15;
  QSlider *dilateSlider;
  QSpinBox *dilateLabel;

};

#endif // THRESHOLDWIDGET_H
