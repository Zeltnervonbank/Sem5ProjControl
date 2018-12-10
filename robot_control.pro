TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    marbel_controller.cpp \
    camera.cpp \
    lidar.cpp \
    mapping.cpp \
    movement.cpp \
    waypointnavigation.cpp \
    pathing.cpp \
    wall_controller.cpp \
    qlearning.cpp \
    waypointcontroller.cpp

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
    camera.h \
    datatypes.h \
    lidar.h \
    mapping.h \
    movement.h \
    waypointnavigation.h \
    globals.h \
    pathing.h \
    wall_controller.h \
    qlearning.h \
    waypointcontroller.h

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin/release/ -lfuzzylite
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin/debug/ -lfuzzylite
else:unix: LIBS += -L$$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin
DEPENDPATH += $$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin
@LIBS += -L$$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin
INCLUDEPATH +=$$PWD/../../../fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/


unix:!macx: LIBS += -L$$PWD/../../fuzzylite-6.0/fuzzylite/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/../../fuzzylite-6.0/fuzzylite/release/bin
DEPENDPATH += $$PWD/../../fuzzylite-6.0/fuzzylite/release/bin
@LIBS += -L$$PWD/../../fuzzylite-6.0/fuzzylite/release/bin
INCLUDEPATH +=$$PWD/../../fuzzylite-6.0/fuzzylite/

unix:!macx: LIBS += -L$$PWD/../../../../fuzzylite-6.0/fuzzylite/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/../../../../fuzzylite-6.0/fuzzylite/release/bin
DEPENDPATH += $$PWD/../../../../fuzzylite-6.0/fuzzylite/release/bin
@LIBS += -L$$PWD/../../../../fuzzylite-6.0/fuzzylite/release/bin
INCLUDEPATH +=$$PWD/../../../../fuzzylite-6.0/fuzzylite

unix:!macx: LIBS += -L$$PWD/../../../../fuzzylite-6.0/fuzzylite/release/bin/ -lfuzzylite

INCLUDEPATH += $$PWD/../../fuzzylite-6.0/fuzzylite/release/bin
DEPENDPATH += $$PWD/../../fuzzylite-6.0/fuzzylite/release/bin
@LIBS += -L$$PWD/../../fuzzylite-6.0/fuzzylite/release/bin
INCLUDEPATH +=$$PWD/../../fuzzylite-6.0/fuzzylite
