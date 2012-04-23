cmake_minimum_required(VERSION 2.4)
PROJECT (MRP_homework)

SET (CMAKE_MODULE_PATH "~/Documents/School/20113/mrp/cmake/")
SET (CPP_FLAGS "-g")
INCLUDE (UsePlayerC++)
find_package(GLUT)
find_package(OpenGL)
link_directories(/usr/X11/lib)

PLAYER_ADD_PLAYERCPP_CLIENT (mapper SOURCES mapper.cpp LINKFLAGS)
target_link_libraries(mapper ${GLUT_LIBRARY} ${OPENGL_LIBRARY} X11 Xext)
