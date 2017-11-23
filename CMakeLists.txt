cmake_minimum_required(VERSION 2.8)
project(LianSearch)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
set(SOURCE_FILES
	main.cpp
    	tinyxml/tinyxmlparser.cpp
    	tinyxml/tinyxmlerror.cpp
    	tinyxml/tinyxml.cpp
   	tinyxml/tinystr.cpp
    	liansearch.cpp
    	openlist.cpp
    	config.cpp
    	map.cpp
    	mission.cpp
    	xmllogger.cpp)

set(HEADER_FILES
	tinyxml/tinyxml.h
    	tinyxml/tinystr.h
    	searchresult.h
   	gl_const.h
   	liansearch.h
   	node.h
   	openlist.h
   	config.h
   	map.h
   	logger.h
   	mission.h
   	search.h
   	xmllogger.h)
add_executable(LianSearch ${SOURCE_FILES} ${HEADER_FILES})
