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


SOURCES += main.cpp\
        mainwindow.cpp \
    ../key_control.cpp \
    ../tcp_socket.cpp \
    ../../common/paramodbus.cpp \
    ../../common/modbus.cpp

HEADERS  += mainwindow.h \
    ../key_control.h \
    ../tcp_socket.h \
    ../../common/paramodbus.h \
    ../../common/modbus.h \
    ../../common/common.h

FORMS    += mainwindow.ui
