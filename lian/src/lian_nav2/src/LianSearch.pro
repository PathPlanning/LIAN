#-------------------------------------------------
#
# Project created by QtCreator 2014-08-11T16:11:48
#
#-------------------------------------------------
#refactoring process

TARGET = LianSearch
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    liansearch.cpp \
    openlist.cpp \
    config.cpp \
    map.cpp \
    mission.cpp \
    xmllogger.cpp \
    tinyxml/tinystr.cpp \
    tinyxml/tinyxml.cpp \
    tinyxml/tinyxmlerror.cpp \
    tinyxml/tinyxmlparser.cpp

HEADERS += \
    searchresult.h \
    gl_const.h \
    liansearch.h \
    node.h \
    openlist.h \
    config.h \
    map.h \
    logger.h \
    mission.h \
    search.h \
    xmllogger.h \
    tinyxml/tinystr.h \
    tinyxml/tinyxml.h
