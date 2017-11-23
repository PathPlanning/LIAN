#-------------------------------------------------
#
# Project created by QtCreator 2014-08-11T16:11:48
#
#-------------------------------------------------

TARGET = LianSearch
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x
TEMPLATE = app
win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}
SOURCES += main.cpp \
    tinyxmlparser.cpp \
    tinyxmlerror.cpp \
    tinyxml.cpp \
    tinystr.cpp \
    cXmlLogger.cpp \
    cMission.cpp \
    cMap.cpp \
    cLogger.cpp \
    cList.cpp \
    cConfig.cpp \
    liansearch.cpp

HEADERS += \
    tinyxml.h \
    tinystr.h \
    sNode.h \
    searchresult.h \
    gl_const.h \
    cXmlLogger.h \
    cSearch.h \
    cMission.h \
    cMap.h \
    cLogger.h \
    cList.h \
    cConfig.h \
    liansearch.h
