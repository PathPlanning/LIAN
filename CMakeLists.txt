cmake_minimum_required(VERSION 2.8)
project(LianSearch)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
set(SOURCE_FILES
	    src/tech_debt/main.cpp
    	tinyxml/tinyxmlparser.cpp
    	tinyxml/tinyxmlerror.cpp
    	tinyxml/tinyxml.cpp
        tinyxml/tinystr.cpp
    	src/tech_debt/liansearch.cpp
    	src/tech_debt/openlist.cpp
    	src/tech_debt/config.cpp
    	src/tech_debt/map.cpp
    	src/tech_debt/mission.cpp
    	src/tech_debt/xmllogger.cpp)

set(HEADER_FILES
	tinyxml/tinyxml.h
    tinyxml/tinystr.h
  	src/tech_debt/searchresult.h
   	src/tech_debt/gl_const.h
   	src/tech_debt/liansearch.h
   	src/tech_debt/node.h
   	src/tech_debt/openlist.h
   	src/tech_debt/config.h
   	src/tech_debt/map.h
   	src/tech_debt/logger.h
   	src/tech_debt/mission.h
   	src/tech_debt/search.h
   	src/tech_debt/xmllogger.h)
add_executable(LianSearch src/${SOURCE_FILES} ${HEADER_FILES})
