/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *leftLayout;
    QGroupBox *groupBoxData;
    QFormLayout *formLayout;
    QLabel *labelRope;
    QLabel *labelRopeValue;
    QLabel *labelEncoder;
    QLabel *labelEncoderValue;
    QLabel *labelCurrent;
    QLabel *labelCurrentValue;
    QLabel *labelVoltage;
    QLabel *labelVoltageValue;
    QLabel *labelMotorSpeed;
    QLabel *labelMotorSpeedValue;
    QLabel *labelMotorPosition;
    QLabel *labelMotorPositionValue;
    QLabel *labelMotorStatus;
    QLabel *labelMotorStatusValue;
    QLabel *labelPressure;
    QLabel *labelPressureValue;
    QLabel *labelDataCount;
    QLabel *labelDataCountValue;
    QGroupBox *groupBoxMotorControl;
    QVBoxLayout *verticalLayoutMotor;
    QHBoxLayout *horizontalLayoutVelocity;
    QLabel *labelVelocityInput;
    QDoubleSpinBox *spinBoxVelocity;
    QPushButton *btnSetVelocity;
    QHBoxLayout *horizontalLayoutPosition;
    QLabel *labelPositionInput;
    QDoubleSpinBox *spinBoxPosition;
    QPushButton *btnSetPosition;
    QHBoxLayout *horizontalLayoutMotorBasic;
    QPushButton *btnMotorEnable;
    QPushButton *btnMotorDisable;
    QPushButton *btnMotorStop;
    QGroupBox *groupBoxPowerControl;
    QVBoxLayout *verticalLayoutPower;
    QHBoxLayout *horizontalLayoutPowerCurrent;
    QLabel *labelPowerCurrent;
    QDoubleSpinBox *spinBoxPowerCurrent;
    QPushButton *btnSetPowerCurrent;
    QHBoxLayout *horizontalLayoutPowerVoltage;
    QLabel *labelPowerVoltage;
    QDoubleSpinBox *spinBoxPowerVoltage;
    QPushButton *btnSetPowerVoltage;
    QHBoxLayout *horizontalLayoutPowerOnOff;
    QPushButton *btnPowerOn;
    QPushButton *btnPowerOff;
    QGroupBox *groupBoxControl;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *labelDataPath;
    QLineEdit *lineEditDataPath;
    QPushButton *btnBrowse;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *btnStart;
    QPushButton *btnStop;
    QHBoxLayout *horizontalLayout_4;
    QLabel *labelCollectStatusTitle;
    QLabel *labelCollectStatus;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *btnRecordStart;
    QPushButton *btnRecordStop;
    QHBoxLayout *horizontalLayout_6;
    QLabel *labelRecordStatusTitle;
    QLabel *labelRecordStatus;
    QHBoxLayout *horizontalLayout_7;
    QLabel *labelDisplayType;
    QComboBox *comboBoxDisplayType;
    QPushButton *btnLoadHistory;
    QSpacerItem *verticalSpacer;
    QWidget *chartContainer;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1400, 900);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        leftLayout = new QVBoxLayout();
        leftLayout->setObjectName(QString::fromUtf8("leftLayout"));
        groupBoxData = new QGroupBox(centralwidget);
        groupBoxData->setObjectName(QString::fromUtf8("groupBoxData"));
        formLayout = new QFormLayout(groupBoxData);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        labelRope = new QLabel(groupBoxData);
        labelRope->setObjectName(QString::fromUtf8("labelRope"));

        formLayout->setWidget(0, QFormLayout::LabelRole, labelRope);

        labelRopeValue = new QLabel(groupBoxData);
        labelRopeValue->setObjectName(QString::fromUtf8("labelRopeValue"));
        labelRopeValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: green;"));

        formLayout->setWidget(0, QFormLayout::FieldRole, labelRopeValue);

        labelEncoder = new QLabel(groupBoxData);
        labelEncoder->setObjectName(QString::fromUtf8("labelEncoder"));

        formLayout->setWidget(1, QFormLayout::LabelRole, labelEncoder);

        labelEncoderValue = new QLabel(groupBoxData);
        labelEncoderValue->setObjectName(QString::fromUtf8("labelEncoderValue"));
        labelEncoderValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: darkBlue;"));

        formLayout->setWidget(1, QFormLayout::FieldRole, labelEncoderValue);

        labelCurrent = new QLabel(groupBoxData);
        labelCurrent->setObjectName(QString::fromUtf8("labelCurrent"));

        formLayout->setWidget(2, QFormLayout::LabelRole, labelCurrent);

        labelCurrentValue = new QLabel(groupBoxData);
        labelCurrentValue->setObjectName(QString::fromUtf8("labelCurrentValue"));
        labelCurrentValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: red;"));

        formLayout->setWidget(2, QFormLayout::FieldRole, labelCurrentValue);

        labelVoltage = new QLabel(groupBoxData);
        labelVoltage->setObjectName(QString::fromUtf8("labelVoltage"));

        formLayout->setWidget(3, QFormLayout::LabelRole, labelVoltage);

        labelVoltageValue = new QLabel(groupBoxData);
        labelVoltageValue->setObjectName(QString::fromUtf8("labelVoltageValue"));
        labelVoltageValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: magenta;"));

        formLayout->setWidget(3, QFormLayout::FieldRole, labelVoltageValue);

        labelMotorSpeed = new QLabel(groupBoxData);
        labelMotorSpeed->setObjectName(QString::fromUtf8("labelMotorSpeed"));

        formLayout->setWidget(4, QFormLayout::LabelRole, labelMotorSpeed);

        labelMotorSpeedValue = new QLabel(groupBoxData);
        labelMotorSpeedValue->setObjectName(QString::fromUtf8("labelMotorSpeedValue"));
        labelMotorSpeedValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: orange;"));

        formLayout->setWidget(4, QFormLayout::FieldRole, labelMotorSpeedValue);

        labelMotorPosition = new QLabel(groupBoxData);
        labelMotorPosition->setObjectName(QString::fromUtf8("labelMotorPosition"));

        formLayout->setWidget(5, QFormLayout::LabelRole, labelMotorPosition);

        labelMotorPositionValue = new QLabel(groupBoxData);
        labelMotorPositionValue->setObjectName(QString::fromUtf8("labelMotorPositionValue"));
        labelMotorPositionValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: purple;"));

        formLayout->setWidget(5, QFormLayout::FieldRole, labelMotorPositionValue);

        labelMotorStatus = new QLabel(groupBoxData);
        labelMotorStatus->setObjectName(QString::fromUtf8("labelMotorStatus"));

        formLayout->setWidget(6, QFormLayout::LabelRole, labelMotorStatus);

        labelMotorStatusValue = new QLabel(groupBoxData);
        labelMotorStatusValue->setObjectName(QString::fromUtf8("labelMotorStatusValue"));
        labelMotorStatusValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: gray;"));

        formLayout->setWidget(6, QFormLayout::FieldRole, labelMotorStatusValue);

        labelPressure = new QLabel(groupBoxData);
        labelPressure->setObjectName(QString::fromUtf8("labelPressure"));

        formLayout->setWidget(7, QFormLayout::LabelRole, labelPressure);

        labelPressureValue = new QLabel(groupBoxData);
        labelPressureValue->setObjectName(QString::fromUtf8("labelPressureValue"));
        labelPressureValue->setStyleSheet(QString::fromUtf8("font-weight: bold; color: blue;"));

        formLayout->setWidget(7, QFormLayout::FieldRole, labelPressureValue);

        labelDataCount = new QLabel(groupBoxData);
        labelDataCount->setObjectName(QString::fromUtf8("labelDataCount"));

        formLayout->setWidget(8, QFormLayout::LabelRole, labelDataCount);

        labelDataCountValue = new QLabel(groupBoxData);
        labelDataCountValue->setObjectName(QString::fromUtf8("labelDataCountValue"));

        formLayout->setWidget(8, QFormLayout::FieldRole, labelDataCountValue);


        leftLayout->addWidget(groupBoxData);

        groupBoxMotorControl = new QGroupBox(centralwidget);
        groupBoxMotorControl->setObjectName(QString::fromUtf8("groupBoxMotorControl"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBoxMotorControl->sizePolicy().hasHeightForWidth());
        groupBoxMotorControl->setSizePolicy(sizePolicy);
        verticalLayoutMotor = new QVBoxLayout(groupBoxMotorControl);
        verticalLayoutMotor->setObjectName(QString::fromUtf8("verticalLayoutMotor"));
        horizontalLayoutVelocity = new QHBoxLayout();
        horizontalLayoutVelocity->setObjectName(QString::fromUtf8("horizontalLayoutVelocity"));
        labelVelocityInput = new QLabel(groupBoxMotorControl);
        labelVelocityInput->setObjectName(QString::fromUtf8("labelVelocityInput"));

        horizontalLayoutVelocity->addWidget(labelVelocityInput);

        spinBoxVelocity = new QDoubleSpinBox(groupBoxMotorControl);
        spinBoxVelocity->setObjectName(QString::fromUtf8("spinBoxVelocity"));
        spinBoxVelocity->setMinimum(-10000.000000000000000);
        spinBoxVelocity->setMaximum(10000.000000000000000);
        spinBoxVelocity->setSingleStep(100.000000000000000);
        spinBoxVelocity->setValue(0.000000000000000);

        horizontalLayoutVelocity->addWidget(spinBoxVelocity);

        btnSetVelocity = new QPushButton(groupBoxMotorControl);
        btnSetVelocity->setObjectName(QString::fromUtf8("btnSetVelocity"));

        horizontalLayoutVelocity->addWidget(btnSetVelocity);


        verticalLayoutMotor->addLayout(horizontalLayoutVelocity);

        horizontalLayoutPosition = new QHBoxLayout();
        horizontalLayoutPosition->setObjectName(QString::fromUtf8("horizontalLayoutPosition"));
        labelPositionInput = new QLabel(groupBoxMotorControl);
        labelPositionInput->setObjectName(QString::fromUtf8("labelPositionInput"));

        horizontalLayoutPosition->addWidget(labelPositionInput);

        spinBoxPosition = new QDoubleSpinBox(groupBoxMotorControl);
        spinBoxPosition->setObjectName(QString::fromUtf8("spinBoxPosition"));
        spinBoxPosition->setMinimum(-999999.000000000000000);
        spinBoxPosition->setMaximum(999999.000000000000000);
        spinBoxPosition->setSingleStep(1000.000000000000000);
        spinBoxPosition->setValue(0.000000000000000);

        horizontalLayoutPosition->addWidget(spinBoxPosition);

        btnSetPosition = new QPushButton(groupBoxMotorControl);
        btnSetPosition->setObjectName(QString::fromUtf8("btnSetPosition"));

        horizontalLayoutPosition->addWidget(btnSetPosition);


        verticalLayoutMotor->addLayout(horizontalLayoutPosition);

        horizontalLayoutMotorBasic = new QHBoxLayout();
        horizontalLayoutMotorBasic->setObjectName(QString::fromUtf8("horizontalLayoutMotorBasic"));
        btnMotorEnable = new QPushButton(groupBoxMotorControl);
        btnMotorEnable->setObjectName(QString::fromUtf8("btnMotorEnable"));

        horizontalLayoutMotorBasic->addWidget(btnMotorEnable);

        btnMotorDisable = new QPushButton(groupBoxMotorControl);
        btnMotorDisable->setObjectName(QString::fromUtf8("btnMotorDisable"));

        horizontalLayoutMotorBasic->addWidget(btnMotorDisable);

        btnMotorStop = new QPushButton(groupBoxMotorControl);
        btnMotorStop->setObjectName(QString::fromUtf8("btnMotorStop"));
        btnMotorStop->setStyleSheet(QString::fromUtf8("background-color: red; color: white;"));

        horizontalLayoutMotorBasic->addWidget(btnMotorStop);


        verticalLayoutMotor->addLayout(horizontalLayoutMotorBasic);


        leftLayout->addWidget(groupBoxMotorControl);

        groupBoxPowerControl = new QGroupBox(centralwidget);
        groupBoxPowerControl->setObjectName(QString::fromUtf8("groupBoxPowerControl"));
        sizePolicy.setHeightForWidth(groupBoxPowerControl->sizePolicy().hasHeightForWidth());
        groupBoxPowerControl->setSizePolicy(sizePolicy);
        verticalLayoutPower = new QVBoxLayout(groupBoxPowerControl);
        verticalLayoutPower->setObjectName(QString::fromUtf8("verticalLayoutPower"));
        horizontalLayoutPowerCurrent = new QHBoxLayout();
        horizontalLayoutPowerCurrent->setObjectName(QString::fromUtf8("horizontalLayoutPowerCurrent"));
        labelPowerCurrent = new QLabel(groupBoxPowerControl);
        labelPowerCurrent->setObjectName(QString::fromUtf8("labelPowerCurrent"));

        horizontalLayoutPowerCurrent->addWidget(labelPowerCurrent);

        spinBoxPowerCurrent = new QDoubleSpinBox(groupBoxPowerControl);
        spinBoxPowerCurrent->setObjectName(QString::fromUtf8("spinBoxPowerCurrent"));
        spinBoxPowerCurrent->setMinimum(0.050000000000000);
        spinBoxPowerCurrent->setMaximum(4.000000000000000);
        spinBoxPowerCurrent->setSingleStep(0.100000000000000);
        spinBoxPowerCurrent->setValue(0.500000000000000);

        horizontalLayoutPowerCurrent->addWidget(spinBoxPowerCurrent);

        btnSetPowerCurrent = new QPushButton(groupBoxPowerControl);
        btnSetPowerCurrent->setObjectName(QString::fromUtf8("btnSetPowerCurrent"));

        horizontalLayoutPowerCurrent->addWidget(btnSetPowerCurrent);


        verticalLayoutPower->addLayout(horizontalLayoutPowerCurrent);

        horizontalLayoutPowerVoltage = new QHBoxLayout();
        horizontalLayoutPowerVoltage->setObjectName(QString::fromUtf8("horizontalLayoutPowerVoltage"));
        labelPowerVoltage = new QLabel(groupBoxPowerControl);
        labelPowerVoltage->setObjectName(QString::fromUtf8("labelPowerVoltage"));

        horizontalLayoutPowerVoltage->addWidget(labelPowerVoltage);

        spinBoxPowerVoltage = new QDoubleSpinBox(groupBoxPowerControl);
        spinBoxPowerVoltage->setObjectName(QString::fromUtf8("spinBoxPowerVoltage"));
        spinBoxPowerVoltage->setMinimum(0.000000000000000);
        spinBoxPowerVoltage->setMaximum(30.000000000000000);
        spinBoxPowerVoltage->setSingleStep(0.500000000000000);
        spinBoxPowerVoltage->setValue(12.000000000000000);

        horizontalLayoutPowerVoltage->addWidget(spinBoxPowerVoltage);

        btnSetPowerVoltage = new QPushButton(groupBoxPowerControl);
        btnSetPowerVoltage->setObjectName(QString::fromUtf8("btnSetPowerVoltage"));

        horizontalLayoutPowerVoltage->addWidget(btnSetPowerVoltage);


        verticalLayoutPower->addLayout(horizontalLayoutPowerVoltage);

        horizontalLayoutPowerOnOff = new QHBoxLayout();
        horizontalLayoutPowerOnOff->setObjectName(QString::fromUtf8("horizontalLayoutPowerOnOff"));
        btnPowerOn = new QPushButton(groupBoxPowerControl);
        btnPowerOn->setObjectName(QString::fromUtf8("btnPowerOn"));
        btnPowerOn->setStyleSheet(QString::fromUtf8("background-color: green; color: white;"));

        horizontalLayoutPowerOnOff->addWidget(btnPowerOn);

        btnPowerOff = new QPushButton(groupBoxPowerControl);
        btnPowerOff->setObjectName(QString::fromUtf8("btnPowerOff"));
        btnPowerOff->setStyleSheet(QString::fromUtf8("background-color: red; color: white;"));

        horizontalLayoutPowerOnOff->addWidget(btnPowerOff);


        verticalLayoutPower->addLayout(horizontalLayoutPowerOnOff);


        leftLayout->addWidget(groupBoxPowerControl);

        groupBoxControl = new QGroupBox(centralwidget);
        groupBoxControl->setObjectName(QString::fromUtf8("groupBoxControl"));
        verticalLayout_2 = new QVBoxLayout(groupBoxControl);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        labelDataPath = new QLabel(groupBoxControl);
        labelDataPath->setObjectName(QString::fromUtf8("labelDataPath"));

        horizontalLayout_2->addWidget(labelDataPath);

        lineEditDataPath = new QLineEdit(groupBoxControl);
        lineEditDataPath->setObjectName(QString::fromUtf8("lineEditDataPath"));

        horizontalLayout_2->addWidget(lineEditDataPath);

        btnBrowse = new QPushButton(groupBoxControl);
        btnBrowse->setObjectName(QString::fromUtf8("btnBrowse"));

        horizontalLayout_2->addWidget(btnBrowse);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        btnStart = new QPushButton(groupBoxControl);
        btnStart->setObjectName(QString::fromUtf8("btnStart"));
        btnStart->setStyleSheet(QString::fromUtf8("background-color: green; color: white;"));

        horizontalLayout_3->addWidget(btnStart);

        btnStop = new QPushButton(groupBoxControl);
        btnStop->setObjectName(QString::fromUtf8("btnStop"));
        btnStop->setStyleSheet(QString::fromUtf8("background-color: red; color: white;"));

        horizontalLayout_3->addWidget(btnStop);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        labelCollectStatusTitle = new QLabel(groupBoxControl);
        labelCollectStatusTitle->setObjectName(QString::fromUtf8("labelCollectStatusTitle"));

        horizontalLayout_4->addWidget(labelCollectStatusTitle);

        labelCollectStatus = new QLabel(groupBoxControl);
        labelCollectStatus->setObjectName(QString::fromUtf8("labelCollectStatus"));

        horizontalLayout_4->addWidget(labelCollectStatus);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        btnRecordStart = new QPushButton(groupBoxControl);
        btnRecordStart->setObjectName(QString::fromUtf8("btnRecordStart"));

        horizontalLayout_5->addWidget(btnRecordStart);

        btnRecordStop = new QPushButton(groupBoxControl);
        btnRecordStop->setObjectName(QString::fromUtf8("btnRecordStop"));

        horizontalLayout_5->addWidget(btnRecordStop);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        labelRecordStatusTitle = new QLabel(groupBoxControl);
        labelRecordStatusTitle->setObjectName(QString::fromUtf8("labelRecordStatusTitle"));

        horizontalLayout_6->addWidget(labelRecordStatusTitle);

        labelRecordStatus = new QLabel(groupBoxControl);
        labelRecordStatus->setObjectName(QString::fromUtf8("labelRecordStatus"));

        horizontalLayout_6->addWidget(labelRecordStatus);


        verticalLayout_2->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        labelDisplayType = new QLabel(groupBoxControl);
        labelDisplayType->setObjectName(QString::fromUtf8("labelDisplayType"));

        horizontalLayout_7->addWidget(labelDisplayType);

        comboBoxDisplayType = new QComboBox(groupBoxControl);
        comboBoxDisplayType->setObjectName(QString::fromUtf8("comboBoxDisplayType"));

        horizontalLayout_7->addWidget(comboBoxDisplayType);


        verticalLayout_2->addLayout(horizontalLayout_7);

        btnLoadHistory = new QPushButton(groupBoxControl);
        btnLoadHistory->setObjectName(QString::fromUtf8("btnLoadHistory"));

        verticalLayout_2->addWidget(btnLoadHistory);


        leftLayout->addWidget(groupBoxControl);

        verticalSpacer = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        leftLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(leftLayout);

        chartContainer = new QWidget(centralwidget);
        chartContainer->setObjectName(QString::fromUtf8("chartContainer"));
        chartContainer->setMinimumSize(QSize(900, 700));

        horizontalLayout->addWidget(chartContainer);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1400, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\351\207\215\345\212\233\345\215\270\350\275\275\347\263\273\347\273\237\346\225\260\346\215\256\347\233\221\346\216\247 - GravShow", nullptr));
        groupBoxData->setTitle(QApplication::translate("MainWindow", "\345\256\236\346\227\266\346\225\260\346\215\256", nullptr));
        labelRope->setText(QApplication::translate("MainWindow", "\347\273\263\351\225\277:", nullptr));
        labelRopeValue->setText(QApplication::translate("MainWindow", "0.000 m", nullptr));
        labelEncoder->setText(QApplication::translate("MainWindow", "\347\274\226\347\240\201\345\231\250:", nullptr));
        labelEncoderValue->setText(QApplication::translate("MainWindow", "0", nullptr));
        labelCurrent->setText(QApplication::translate("MainWindow", "\347\224\265\346\265\201:", nullptr));
        labelCurrentValue->setText(QApplication::translate("MainWindow", "0.000 A", nullptr));
        labelVoltage->setText(QApplication::translate("MainWindow", "\347\224\265\345\216\213:", nullptr));
        labelVoltageValue->setText(QApplication::translate("MainWindow", "0.00 V", nullptr));
        labelMotorSpeed->setText(QApplication::translate("MainWindow", "\347\224\265\346\234\272\351\200\237\345\272\246:", nullptr));
        labelMotorSpeedValue->setText(QApplication::translate("MainWindow", "0 rpm", nullptr));
        labelMotorPosition->setText(QApplication::translate("MainWindow", "\347\224\265\346\234\272\344\275\215\347\275\256:", nullptr));
        labelMotorPositionValue->setText(QApplication::translate("MainWindow", "0", nullptr));
        labelMotorStatus->setText(QApplication::translate("MainWindow", "\347\224\265\346\234\272\347\212\266\346\200\201:", nullptr));
        labelMotorStatusValue->setText(QApplication::translate("MainWindow", "Stopped", nullptr));
        labelPressure->setText(QApplication::translate("MainWindow", "\345\216\213\345\212\233:", nullptr));
        labelPressureValue->setText(QApplication::translate("MainWindow", "0.000 kg", nullptr));
        labelDataCount->setText(QApplication::translate("MainWindow", "\346\225\260\346\215\256\347\202\271\346\225\260:", nullptr));
        labelDataCountValue->setText(QApplication::translate("MainWindow", "0", nullptr));
        groupBoxMotorControl->setTitle(QApplication::translate("MainWindow", "\347\224\265\346\234\272\346\216\247\345\210\266\357\274\210\347\256\227\346\263\225\350\277\220\350\241\214\345\211\215\357\274\211", nullptr));
        labelVelocityInput->setText(QApplication::translate("MainWindow", "\351\200\237\345\272\246(rpm):", nullptr));
        btnSetVelocity->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\351\200\237\345\272\246", nullptr));
        labelPositionInput->setText(QApplication::translate("MainWindow", "\344\275\215\347\275\256:", nullptr));
        btnSetPosition->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\344\275\215\347\275\256", nullptr));
        btnMotorEnable->setText(QApplication::translate("MainWindow", "\344\275\277\350\203\275\347\224\265\346\234\272", nullptr));
        btnMotorDisable->setText(QApplication::translate("MainWindow", "\345\244\261\350\203\275\347\224\265\346\234\272", nullptr));
        btnMotorStop->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242", nullptr));
        groupBoxPowerControl->setTitle(QApplication::translate("MainWindow", "\347\224\265\346\272\220\346\235\277\346\216\247\345\210\266", nullptr));
        labelPowerCurrent->setText(QApplication::translate("MainWindow", "\347\224\265\346\265\201(A):", nullptr));
        btnSetPowerCurrent->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\347\224\265\346\265\201", nullptr));
        labelPowerVoltage->setText(QApplication::translate("MainWindow", "\347\224\265\345\216\213(V):", nullptr));
        btnSetPowerVoltage->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256\347\224\265\345\216\213", nullptr));
        btnPowerOn->setText(QApplication::translate("MainWindow", "\347\224\265\346\272\220\345\274\200\345\220\257", nullptr));
        btnPowerOff->setText(QApplication::translate("MainWindow", "\347\224\265\346\272\220\345\205\263\351\227\255", nullptr));
        groupBoxControl->setTitle(QApplication::translate("MainWindow", "\346\225\260\346\215\256\351\207\207\351\233\206\346\216\247\345\210\266", nullptr));
        labelDataPath->setText(QApplication::translate("MainWindow", "\346\225\260\346\215\256\346\272\220:", nullptr));
        lineEditDataPath->setText(QApplication::translate("MainWindow", "/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/share/realtime_data.txt", nullptr));
        btnBrowse->setText(QApplication::translate("MainWindow", "\346\265\217\350\247\210", nullptr));
        btnStart->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213\351\207\207\351\233\206", nullptr));
        btnStop->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242\351\207\207\351\233\206", nullptr));
        labelCollectStatusTitle->setText(QApplication::translate("MainWindow", "\351\207\207\351\233\206\347\212\266\346\200\201:", nullptr));
        labelCollectStatus->setText(QApplication::translate("MainWindow", "\346\234\252\350\277\236\346\216\245", nullptr));
        btnRecordStart->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213\350\256\260\345\275\225", nullptr));
        btnRecordStop->setText(QApplication::translate("MainWindow", "\345\201\234\346\255\242\350\256\260\345\275\225", nullptr));
        labelRecordStatusTitle->setText(QApplication::translate("MainWindow", "\350\256\260\345\275\225\347\212\266\346\200\201:", nullptr));
        labelRecordStatus->setText(QApplication::translate("MainWindow", "\346\234\252\350\256\260\345\275\225", nullptr));
        labelDisplayType->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\347\261\273\345\236\213:", nullptr));
        btnLoadHistory->setText(QApplication::translate("MainWindow", "\345\212\240\350\275\275\345\216\206\345\217\262\346\225\260\346\215\256", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
