/********************************************************************************
** Form generated from reading UI file 'SamplePlugin.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SAMPLEPLUGIN_H
#define UI_SAMPLEPLUGIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SamplePlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QPushButton *_btn_im;
    QPushButton *_btn_scan;
    QPushButton *_btn_Pose;
    QPushButton *_btn_home;
    QPushButton *_btn_move_default;
    QPushButton *_btn0;
    QPushButton *_btn1;
    QPushButton *_btn2;
    QPushButton *_btn3;
    QPushButton *_btn4;
    QSpinBox *_spinBox;
    QSlider *_slider;
    QFrame *line;
    QPushButton *_btnStop;
    QPushButton *_btnRestart;
    QPushButton *_btnNextFrame;
    QLabel *_label;

    void setupUi(QDockWidget *SamplePlugin)
    {
        if (SamplePlugin->objectName().isEmpty())
            SamplePlugin->setObjectName(QString::fromUtf8("SamplePlugin"));
        SamplePlugin->resize(476, 479);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(8);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SamplePlugin->sizePolicy().hasHeightForWidth());
        SamplePlugin->setSizePolicy(sizePolicy);
        SamplePlugin->setStyleSheet(QString::fromUtf8(" QToolTip\n"
"{\n"
"border: 1px solid black;\n"
"background-color: #ffa02f; \n"
"padding: 1px;\n"
"border-radius: 3px;\n"
"opacity: 100;\n"
"}\n"
""));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _btn_im = new QPushButton(dockWidgetContents);
        _btn_im->setObjectName(QString::fromUtf8("_btn_im"));

        verticalLayout->addWidget(_btn_im);

        _btn_scan = new QPushButton(dockWidgetContents);
        _btn_scan->setObjectName(QString::fromUtf8("_btn_scan"));

        verticalLayout->addWidget(_btn_scan);

        _btn_Pose = new QPushButton(dockWidgetContents);
        _btn_Pose->setObjectName(QString::fromUtf8("_btn_Pose"));

        verticalLayout->addWidget(_btn_Pose);

        _btn_home = new QPushButton(dockWidgetContents);
        _btn_home->setObjectName(QString::fromUtf8("_btn_home"));

        verticalLayout->addWidget(_btn_home);

        _btn_move_default = new QPushButton(dockWidgetContents);
        _btn_move_default->setObjectName(QString::fromUtf8("_btn_move_default"));

        verticalLayout->addWidget(_btn_move_default);

        _btn0 = new QPushButton(dockWidgetContents);
        _btn0->setObjectName(QString::fromUtf8("_btn0"));

        verticalLayout->addWidget(_btn0);

        _btn1 = new QPushButton(dockWidgetContents);
        _btn1->setObjectName(QString::fromUtf8("_btn1"));

        verticalLayout->addWidget(_btn1);

        _btn2 = new QPushButton(dockWidgetContents);
        _btn2->setObjectName(QString::fromUtf8("_btn2"));

        verticalLayout->addWidget(_btn2);

        _btn3 = new QPushButton(dockWidgetContents);
        _btn3->setObjectName(QString::fromUtf8("_btn3"));

        verticalLayout->addWidget(_btn3);

        _btn4 = new QPushButton(dockWidgetContents);
        _btn4->setObjectName(QString::fromUtf8("_btn4"));

        verticalLayout->addWidget(_btn4);

        _spinBox = new QSpinBox(dockWidgetContents);
        _spinBox->setObjectName(QString::fromUtf8("_spinBox"));

        verticalLayout->addWidget(_spinBox);

        _slider = new QSlider(dockWidgetContents);
        _slider->setObjectName(QString::fromUtf8("_slider"));
        _slider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(_slider);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        _btnStop = new QPushButton(dockWidgetContents);
        _btnStop->setObjectName(QString::fromUtf8("_btnStop"));

        verticalLayout->addWidget(_btnStop);

        _btnRestart = new QPushButton(dockWidgetContents);
        _btnRestart->setObjectName(QString::fromUtf8("_btnRestart"));

        verticalLayout->addWidget(_btnRestart);

        _btnNextFrame = new QPushButton(dockWidgetContents);
        _btnNextFrame->setObjectName(QString::fromUtf8("_btnNextFrame"));

        verticalLayout->addWidget(_btnNextFrame);

        _label = new QLabel(dockWidgetContents);
        _label->setObjectName(QString::fromUtf8("_label"));

        verticalLayout->addWidget(_label);


        verticalLayout_2->addLayout(verticalLayout);

        SamplePlugin->setWidget(dockWidgetContents);

        retranslateUi(SamplePlugin);

        QMetaObject::connectSlotsByName(SamplePlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SamplePlugin)
    {
        SamplePlugin->setWindowTitle(QCoreApplication::translate("SamplePlugin", "RoVi Plugin", nullptr));
        _btn_im->setText(QCoreApplication::translate("SamplePlugin", "Get Image", nullptr));
        _btn_scan->setText(QCoreApplication::translate("SamplePlugin", "Get Scan", nullptr));
        _btn_Pose->setText(QCoreApplication::translate("SamplePlugin", "Get image pose", nullptr));
        _btn_home->setText(QCoreApplication::translate("SamplePlugin", "Move to home", nullptr));
        _btn_move_default->setText(QCoreApplication::translate("SamplePlugin", "Grap test", nullptr));
        _btn0->setText(QCoreApplication::translate("SamplePlugin", "Calculate Path", nullptr));
        _btn1->setText(QCoreApplication::translate("SamplePlugin", "Run Path", nullptr));
        _btn2->setText(QCoreApplication::translate("SamplePlugin", "Go near pick", nullptr));
        _btn3->setText(QCoreApplication::translate("SamplePlugin", "Go pick", nullptr));
        _btn4->setText(QCoreApplication::translate("SamplePlugin", "Test liInt", nullptr));
        _btnStop->setText(QCoreApplication::translate("SamplePlugin", "Stop", nullptr));
        _btnRestart->setText(QCoreApplication::translate("SamplePlugin", "Restart sequence", nullptr));
        _btnNextFrame->setText(QCoreApplication::translate("SamplePlugin", "Next frame", nullptr));
        _label->setText(QCoreApplication::translate("SamplePlugin", "Label", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SamplePlugin: public Ui_SamplePlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SAMPLEPLUGIN_H
