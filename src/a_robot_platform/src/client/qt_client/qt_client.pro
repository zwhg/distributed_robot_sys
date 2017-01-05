#-------------------------------------------------
#
# Project created by QtCreator 2016-12-25T16:02:02
#
#-------------------------------------------------

QT       += core \
            gui \
            network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_client
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++0x
CONFIG += c++11

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += -L/usr/local/lib/ \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc

SOURCES += main.cpp\
        mainwindow.cpp \
    ../key_control.cpp \
    ../tcp_socket.cpp \
    ../../common/paras.cpp \
    ../../common/modbus.cpp

HEADERS  += mainwindow.h \
    ../key_control.h \
    ../tcp_socket.h \
    ../../common/common.h \
    ../../common/paras.h \
    ../../common/modbus.h\
    ../../common/use_display.h
FORMS    += mainwindow.ui
