#include "thresholdwidget.h"

ThresholdWidget::ThresholdWidget(QWidget* parent, Qt::WindowFlags f)
: QGroupBox(parent) {
            this->setObjectName(QStringLiteral("groupBox"));
            this->setGeometry(QRect(0, 0, 540, 250));
            horizontalLayoutWidget = new QWidget(this);
            horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
            horizontalLayoutWidget->setGeometry(QRect(10, 30, 521, 211));
            horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
            horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
            horizontalLayout->setContentsMargins(0, 0, 0, 0);
            formLayout = new QFormLayout();
            formLayout->setObjectName(QStringLiteral("formLayout"));
            label = new QLabel(horizontalLayoutWidget);
            label->setObjectName(QStringLiteral("label"));

            formLayout->setWidget(0, QFormLayout::LabelRole, label);

            hminSlider = new QSlider(horizontalLayoutWidget);
            hminSlider->setObjectName(QStringLiteral("hminSlider"));
            hminSlider->setMinimumSize(QSize(200, 0));
            hminSlider->setOrientation(Qt::Horizontal);

            formLayout->setWidget(1, QFormLayout::LabelRole, hminSlider);

            hminLabel = new QSpinBox(horizontalLayoutWidget);
            hminLabel->setObjectName(QStringLiteral("hminLabel"));
            hminLabel->setMaximumSize(QSize(100, 16777215));

            formLayout->setWidget(1, QFormLayout::FieldRole, hminLabel);

            label_3 = new QLabel(horizontalLayoutWidget);
            label_3->setObjectName(QStringLiteral("label_3"));

            formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

            vminSlider = new QSlider(horizontalLayoutWidget);
            vminSlider->setObjectName(QStringLiteral("vminSlider"));
            vminSlider->setMinimumSize(QSize(200, 0));
            vminSlider->setOrientation(Qt::Horizontal);

            formLayout->setWidget(5, QFormLayout::LabelRole, vminSlider);

            vminLabel = new QSpinBox(horizontalLayoutWidget);
            vminLabel->setObjectName(QStringLiteral("vminLabel"));
            vminLabel->setMaximumSize(QSize(100, 16777215));

            formLayout->setWidget(5, QFormLayout::FieldRole, vminLabel);

            label_5 = new QLabel(horizontalLayoutWidget);
            label_5->setObjectName(QStringLiteral("label_5"));

            formLayout->setWidget(4, QFormLayout::LabelRole, label_5);

            sminSlider = new QSlider(horizontalLayoutWidget);
            sminSlider->setObjectName(QStringLiteral("sminSlider"));
            sminSlider->setMinimumSize(QSize(200, 0));
            sminSlider->setOrientation(Qt::Horizontal);

            formLayout->setWidget(3, QFormLayout::LabelRole, sminSlider);

            sminLabel = new QSpinBox(horizontalLayoutWidget);
            sminLabel->setObjectName(QStringLiteral("sminLabel"));
            sminLabel->setMaximumSize(QSize(100, 16777215));

            formLayout->setWidget(3, QFormLayout::FieldRole, sminLabel);

            label_7 = new QLabel(horizontalLayoutWidget);
            label_7->setObjectName(QStringLiteral("label_7"));

            formLayout->setWidget(6, QFormLayout::LabelRole, label_7);

            erodeSlider = new QSlider(horizontalLayoutWidget);
            erodeSlider->setObjectName(QStringLiteral("erodeSlider"));
            erodeSlider->setMinimumSize(QSize(200, 0));
            erodeSlider->setOrientation(Qt::Horizontal);

            formLayout->setWidget(7, QFormLayout::LabelRole, erodeSlider);

            erodeLabel = new QSpinBox(horizontalLayoutWidget);
            erodeLabel->setObjectName(QStringLiteral("erodeLabel"));
            erodeLabel->setMaximumSize(QSize(100, 16777215));

            formLayout->setWidget(7, QFormLayout::FieldRole, erodeLabel);


            horizontalLayout->addLayout(formLayout);

            formLayout_2 = new QFormLayout();
            formLayout_2->setObjectName(QStringLiteral("formLayout_2"));
            label_9 = new QLabel(horizontalLayoutWidget);
            label_9->setObjectName(QStringLiteral("label_9"));

            formLayout_2->setWidget(0, QFormLayout::LabelRole, label_9);

            hmaxSlider = new QSlider(horizontalLayoutWidget);
            hmaxSlider->setObjectName(QStringLiteral("hmaxSlider"));
            hmaxSlider->setMinimumSize(QSize(200, 0));
            hmaxSlider->setOrientation(Qt::Horizontal);

            formLayout_2->setWidget(1, QFormLayout::LabelRole, hmaxSlider);

            hmaxLabel = new QSpinBox(horizontalLayoutWidget);
            hmaxLabel->setObjectName(QStringLiteral("hmaxLabel"));
            hmaxLabel->setMaximumSize(QSize(100, 16777215));

            formLayout_2->setWidget(1, QFormLayout::FieldRole, hmaxLabel);

            label_11 = new QLabel(horizontalLayoutWidget);
            label_11->setObjectName(QStringLiteral("label_11"));

            formLayout_2->setWidget(2, QFormLayout::LabelRole, label_11);

            vmaxSlider = new QSlider(horizontalLayoutWidget);
            vmaxSlider->setObjectName(QStringLiteral("vmaxSlider"));
            vmaxSlider->setMinimumSize(QSize(200, 0));
            vmaxSlider->setOrientation(Qt::Horizontal);

            formLayout_2->setWidget(5, QFormLayout::LabelRole, vmaxSlider);

            vmaxLabel = new QSpinBox(horizontalLayoutWidget);
            vmaxLabel->setObjectName(QStringLiteral("vmaxLabel"));
            vmaxLabel->setMaximumSize(QSize(100, 16777215));

            formLayout_2->setWidget(5, QFormLayout::FieldRole, vmaxLabel);

            label_13 = new QLabel(horizontalLayoutWidget);
            label_13->setObjectName(QStringLiteral("label_13"));

            formLayout_2->setWidget(4, QFormLayout::LabelRole, label_13);

            smaxSlider = new QSlider(horizontalLayoutWidget);
            smaxSlider->setObjectName(QStringLiteral("smaxSlider"));
            smaxSlider->setMinimumSize(QSize(200, 0));
            smaxSlider->setOrientation(Qt::Horizontal);

            formLayout_2->setWidget(3, QFormLayout::LabelRole, smaxSlider);

            smaxLabel = new QSpinBox(horizontalLayoutWidget);
            smaxLabel->setObjectName(QStringLiteral("smaxLabel"));
            smaxLabel->setMaximumSize(QSize(100, 16777215));

            formLayout_2->setWidget(3, QFormLayout::FieldRole, smaxLabel);

            label_15 = new QLabel(horizontalLayoutWidget);
            label_15->setObjectName(QStringLiteral("label_15"));

            formLayout_2->setWidget(6, QFormLayout::LabelRole, label_15);

            dilateSlider = new QSlider(horizontalLayoutWidget);
            dilateSlider->setObjectName(QStringLiteral("dilateSlider"));
            dilateSlider->setMinimumSize(QSize(200, 0));
            dilateSlider->setOrientation(Qt::Horizontal);

            formLayout_2->setWidget(7, QFormLayout::LabelRole, dilateSlider);

            dilateLabel = new QSpinBox(horizontalLayoutWidget);
            dilateLabel->setObjectName(QStringLiteral("dilateLabel"));
            dilateLabel->setMaximumSize(QSize(100, 16777215));

            formLayout_2->setWidget(7, QFormLayout::FieldRole, dilateLabel);


            horizontalLayout->addLayout(formLayout_2);


            this->setTitle("Thresholds");
            label->setText("Hue Min");
            hminLabel->setValue(0);
            label_3->setText("Sat Min");
            vminLabel->setValue(0);
            label_5->setText("Val Min");
            sminLabel->setValue(0);
            label_7->setText("Erode");
            erodeLabel->setValue(0);
            label_9->setText("Hue Max");
            hmaxLabel->setValue(0);
            label_11->setText("Sat Max");
            vmaxLabel->setValue(0);
            label_13->setText("Val Max");
            smaxLabel->setValue(0);
            label_15->setText("Dilate");
            dilateLabel->setValue(0);

            // set slider's and spinbox's max
            hminSlider->setMaximum(255);
            sminSlider->setMaximum(255);
            vminSlider->setMaximum(255);
            hmaxSlider->setMaximum(255);
            smaxSlider->setMaximum(255);
            vmaxSlider->setMaximum(255);
            erodeSlider->setMaximum(30);
            dilateSlider->setMaximum(30);

            hminLabel->setRange(0,255);
            sminLabel->setRange(0,255);
            vminLabel->setRange(0,255);
            hmaxLabel->setRange(0,255);
            smaxLabel->setRange(0,255);
            vmaxLabel->setRange(0,255);
            erodeLabel->setRange(0,30);
            dilateLabel->setRange(0,30);


            initConnection();
}

void ThresholdWidget::initConnection(){
    connect(hminSlider,     SIGNAL(valueChanged(int)), hminLabel,   SLOT(setValue(int)));
    connect(sminSlider,     SIGNAL(valueChanged(int)), sminLabel,   SLOT(setValue(int)));
    connect(vminSlider,     SIGNAL(valueChanged(int)), vminLabel,   SLOT(setValue(int)));
    connect(hmaxSlider,     SIGNAL(valueChanged(int)), hmaxLabel,   SLOT(setValue(int)));
    connect(smaxSlider,     SIGNAL(valueChanged(int)), smaxLabel,   SLOT(setValue(int)));
    connect(vmaxSlider,     SIGNAL(valueChanged(int)), vmaxLabel,   SLOT(setValue(int)));
    connect(erodeSlider,    SIGNAL(valueChanged(int)), erodeLabel,  SLOT(setValue(int)));
    connect(dilateSlider,   SIGNAL(valueChanged(int)), dilateLabel, SLOT(setValue(int)));
}

void ThresholdWidget::write(QJsonObject &obj) const{
    obj["hmin"] = hminLabel->value();
    obj["smin"] = sminLabel->value();
    obj["vmin"] = vminLabel->value();
    obj["hmax"] = hmaxLabel->value();
    obj["smax"] = smaxLabel->value();
    obj["vmax"] = vmaxLabel->value();
    obj["erode"] = erodeLabel->value();
    obj["dilate"] = dilateLabel->value();
}

void ThresholdWidget::load(const QJsonObject &obj){
    hminSlider->setValue(obj["hmin"].toInt());
    emit hminSlider->valueChanged(obj["hmin"].toInt());
    sminSlider->setValue(obj["smin"].toInt());
    emit sminSlider->valueChanged(obj["smin"].toInt());
    vminSlider->setValue(obj["vmin"].toInt());
    emit vminSlider->valueChanged(obj["vmin"].toInt());
    hmaxSlider->setValue(obj["hmax"].toInt());
    emit hmaxSlider->valueChanged(obj["hmax"].toInt());
    smaxSlider->setValue(obj["smax"].toInt());
    emit smaxSlider->valueChanged(obj["smax"].toInt());
    vmaxSlider->setValue(obj["vmax"].toInt());
    emit vmaxSlider->valueChanged(obj["vmax"].toInt());
    erodeSlider->setValue(obj["erode"].toInt());
    emit erodeSlider->valueChanged(obj["erode"].toInt());
    dilateSlider->setValue(obj["dilate"].toInt());
    emit dilateSlider->valueChanged(obj["dilate"].toInt());
}