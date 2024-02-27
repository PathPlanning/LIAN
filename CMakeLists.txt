cmake_minimum_required(VERSION 3.8)
project(LianSearch)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

add_subdirectory(contrib)

add_executable(LianSearch
		src/main.cpp
		src/liansearch.cpp
		src/structures/search_tree.cpp
		src/structures/config.cpp
		src/structures/map.cpp
		src/structures/mission.cpp
		src/log/logger.cpp
		src/log/xml_utils.cpp

		src/structures/search_result.h
		src/log/gl_const.h
		src/liansearch.h
		src/structures/node.h
		src/structures/search_tree.h
		src/structures/config.h
		src/structures/map.h
		src/log/logger.h
		src/structures/mission.h
		src/log/xml_utils.h)
target_link_libraries(LianSearch PUBLIC tinyxml)
target_include_directories(LianSearch PUBLIC contrib)
