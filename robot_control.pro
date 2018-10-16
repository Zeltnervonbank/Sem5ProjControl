TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    marbel_controller.cpp \
    camera.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv


unix:!macx: LIBS += -L$$PWD/../../../../../Hentet/fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/../../../../../Hentet/fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin
DEPENDPATH += $$PWD/../../../../../Hentet/fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin
@LIBS += -L$$PWD/../../../../../Hentet/fuzzylite-6.0-linux64/fuzzylite-6.0/release/bin
INCLUDEPATH +=/home/alex/Hentet/fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/

HEADERS += \
    marbel_controller.h \
    camera.h

DISTFILES += \
    marbelController.fll
