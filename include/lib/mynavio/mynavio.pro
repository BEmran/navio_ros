#-------------------------------------------------
#
# Project created by QtCreator 2018-03-14T12:50:42
#
#-------------------------------------------------

QT       -= core gui

TARGET = mynavio
TEMPLATE = lib
CONFIG += staticlib

SOURCES += mynavio.cpp

HEADERS += mynavio.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}
