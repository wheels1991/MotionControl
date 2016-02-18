QT += core
QT -= gui

TARGET = MotionControl
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    MotionControler.cpp

HEADERS += \
    MotionControler.h

