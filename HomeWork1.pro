TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += link_pkgconfig
PKGCONFIG += opencv4

INCLUDEPATH += "/media/sf_Tasks/eigen-3.4.0"

SOURCES += \
        main.cpp
