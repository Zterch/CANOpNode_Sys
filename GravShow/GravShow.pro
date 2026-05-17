QT       += core gui widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# 输出目录
DESTDIR = $$PWD/bin

# 目标名称
TARGET = GravShow

DEFINES += QT_DEPRECATED_WARNINGS

# 共享内存需要实时库
LIBS += -lrt

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    datamodel.cpp \
    shmdatacollector.cpp \
    datarecorder.cpp \
    chartwidget.cpp

HEADERS += \
    mainwindow.h \
    datamodel.h \
    shmdatacollector.h \
    datarecorder.h \
    chartwidget.h

FORMS += \
    mainwindow.ui

# 默认部署规则
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
