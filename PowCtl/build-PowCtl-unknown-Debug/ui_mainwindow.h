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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QComboBox *comboBoxPort;
    QLabel *label_2;
    QComboBox *comboBoxBaud;
    QPushButton *pushButtonConnect;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QLabel *label_3;
    QLabel *labelVoltage;
    QLabel *label_5;
    QLabel *labelCurrent;
    QLabel *label_4;
    QLabel *labelTargetVoltage;
    QLabel *label_6;
    QLabel *labelTargetCurrent;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_2;
    QPushButton *pushButtonDec;
    QLineEdit *lineEditCurrent;
    QPushButton *pushButtonInc;
    QPushButton *pushButtonSetCurrent;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_3;
    QPushButton *pushButtonDecVoltage;
    QLineEdit *lineEditVoltage;
    QPushButton *pushButtonIncVoltage;
    QPushButton *pushButtonSetVoltage;
    QTextEdit *textEditLog;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(600, 400);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        horizontalLayout_3 = new QHBoxLayout(groupBox);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_3->addWidget(label);

        comboBoxPort = new QComboBox(groupBox);
        comboBoxPort->addItem(QString());
        comboBoxPort->addItem(QString());
        comboBoxPort->addItem(QString());
        comboBoxPort->setObjectName(QString::fromUtf8("comboBoxPort"));

        horizontalLayout_3->addWidget(comboBoxPort);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_3->addWidget(label_2);

        comboBoxBaud = new QComboBox(groupBox);
        comboBoxBaud->addItem(QString());
        comboBoxBaud->addItem(QString());
        comboBoxBaud->addItem(QString());
        comboBoxBaud->addItem(QString());
        comboBoxBaud->addItem(QString());
        comboBoxBaud->setObjectName(QString::fromUtf8("comboBoxBaud"));

        horizontalLayout_3->addWidget(comboBoxBaud);

        pushButtonConnect = new QPushButton(groupBox);
        pushButtonConnect->setObjectName(QString::fromUtf8("pushButtonConnect"));

        horizontalLayout_3->addWidget(pushButtonConnect);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 0, 0, 1, 1);

        labelVoltage = new QLabel(groupBox_2);
        labelVoltage->setObjectName(QString::fromUtf8("labelVoltage"));
        labelVoltage->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(labelVoltage, 0, 1, 1, 1);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 0, 2, 1, 1);

        labelCurrent = new QLabel(groupBox_2);
        labelCurrent->setObjectName(QString::fromUtf8("labelCurrent"));
        labelCurrent->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(labelCurrent, 0, 3, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        labelTargetVoltage = new QLabel(groupBox_2);
        labelTargetVoltage->setObjectName(QString::fromUtf8("labelTargetVoltage"));
        labelTargetVoltage->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(labelTargetVoltage, 1, 1, 1, 1);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 1, 2, 1, 1);

        labelTargetCurrent = new QLabel(groupBox_2);
        labelTargetCurrent->setObjectName(QString::fromUtf8("labelTargetCurrent"));
        labelTargetCurrent->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(labelTargetCurrent, 1, 3, 1, 1);


        verticalLayout->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(centralwidget);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_2 = new QGridLayout(groupBox_3);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButtonDec = new QPushButton(groupBox_3);
        pushButtonDec->setObjectName(QString::fromUtf8("pushButtonDec"));

        gridLayout_2->addWidget(pushButtonDec, 0, 0, 1, 1);

        lineEditCurrent = new QLineEdit(groupBox_3);
        lineEditCurrent->setObjectName(QString::fromUtf8("lineEditCurrent"));
        lineEditCurrent->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(lineEditCurrent, 0, 1, 1, 1);

        pushButtonInc = new QPushButton(groupBox_3);
        pushButtonInc->setObjectName(QString::fromUtf8("pushButtonInc"));

        gridLayout_2->addWidget(pushButtonInc, 0, 2, 1, 1);

        pushButtonSetCurrent = new QPushButton(groupBox_3);
        pushButtonSetCurrent->setObjectName(QString::fromUtf8("pushButtonSetCurrent"));

        gridLayout_2->addWidget(pushButtonSetCurrent, 0, 3, 1, 1);


        verticalLayout->addWidget(groupBox_3);

        groupBox_4 = new QGroupBox(centralwidget);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        gridLayout_3 = new QGridLayout(groupBox_4);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        pushButtonDecVoltage = new QPushButton(groupBox_4);
        pushButtonDecVoltage->setObjectName(QString::fromUtf8("pushButtonDecVoltage"));

        gridLayout_3->addWidget(pushButtonDecVoltage, 0, 0, 1, 1);

        lineEditVoltage = new QLineEdit(groupBox_4);
        lineEditVoltage->setObjectName(QString::fromUtf8("lineEditVoltage"));
        lineEditVoltage->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(lineEditVoltage, 0, 1, 1, 1);

        pushButtonIncVoltage = new QPushButton(groupBox_4);
        pushButtonIncVoltage->setObjectName(QString::fromUtf8("pushButtonIncVoltage"));

        gridLayout_3->addWidget(pushButtonIncVoltage, 0, 2, 1, 1);

        pushButtonSetVoltage = new QPushButton(groupBox_4);
        pushButtonSetVoltage->setObjectName(QString::fromUtf8("pushButtonSetVoltage"));

        gridLayout_3->addWidget(pushButtonSetVoltage, 0, 3, 1, 1);


        verticalLayout->addWidget(groupBox_4);

        textEditLog = new QTextEdit(centralwidget);
        textEditLog->setObjectName(QString::fromUtf8("textEditLog"));
        textEditLog->setReadOnly(true);

        verticalLayout->addWidget(textEditLog);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\347\224\265\346\272\220\346\235\277\346\216\247\345\210\266", nullptr));
        groupBox->setTitle(QApplication::translate("MainWindow", "\344\270\262\345\217\243\350\256\276\347\275\256", nullptr));
        label->setText(QApplication::translate("MainWindow", "\344\270\262\345\217\243:", nullptr));
        comboBoxPort->setItemText(0, QApplication::translate("MainWindow", "/dev/ttyUSB0", nullptr));
        comboBoxPort->setItemText(1, QApplication::translate("MainWindow", "/dev/ttyUSB1", nullptr));
        comboBoxPort->setItemText(2, QApplication::translate("MainWindow", "/dev/ttyACM0", nullptr));

        label_2->setText(QApplication::translate("MainWindow", "\346\263\242\347\211\271\347\216\207:", nullptr));
        comboBoxBaud->setItemText(0, QApplication::translate("MainWindow", "115200", nullptr));
        comboBoxBaud->setItemText(1, QApplication::translate("MainWindow", "57600", nullptr));
        comboBoxBaud->setItemText(2, QApplication::translate("MainWindow", "38400", nullptr));
        comboBoxBaud->setItemText(3, QApplication::translate("MainWindow", "19200", nullptr));
        comboBoxBaud->setItemText(4, QApplication::translate("MainWindow", "9600", nullptr));

        pushButtonConnect->setText(QApplication::translate("MainWindow", "\350\277\236\346\216\245", nullptr));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "\345\256\236\346\227\266\346\225\260\346\215\256", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "\350\276\223\345\207\272\347\224\265\345\216\213:", nullptr));
        labelVoltage->setText(QApplication::translate("MainWindow", "0.00 V", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "\350\276\223\345\207\272\347\224\265\346\265\201:", nullptr));
        labelCurrent->setText(QApplication::translate("MainWindow", "0.00 A", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "\347\233\256\346\240\207\347\224\265\345\216\213:", nullptr));
        labelTargetVoltage->setText(QApplication::translate("MainWindow", "0.00 V", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "\347\233\256\346\240\207\347\224\265\346\265\201:", nullptr));
        labelTargetCurrent->setText(QApplication::translate("MainWindow", "0.00 A", nullptr));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "\347\224\265\346\265\201\346\216\247\345\210\266", nullptr));
        pushButtonDec->setText(QApplication::translate("MainWindow", "-", nullptr));
        lineEditCurrent->setText(QApplication::translate("MainWindow", "0.44", nullptr));
        pushButtonInc->setText(QApplication::translate("MainWindow", "+", nullptr));
        pushButtonSetCurrent->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256", nullptr));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "\347\224\265\345\216\213\346\216\247\345\210\266", nullptr));
        pushButtonDecVoltage->setText(QApplication::translate("MainWindow", "-", nullptr));
        lineEditVoltage->setText(QApplication::translate("MainWindow", "24.00", nullptr));
        pushButtonIncVoltage->setText(QApplication::translate("MainWindow", "+", nullptr));
        pushButtonSetVoltage->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\256", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
