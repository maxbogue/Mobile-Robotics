cmake_minimum_required(VERSION 2.4)
PROJECT (MRP_homework)

SET (CMAKE_MODULE_PATH "~/Documents/School/20113/mrp/cmake/")
INCLUDE (UsePlayerC++)
find_package(GLUT)
find_package(OpenGL)

PLAYER_ADD_PLAYERCPP_CLIENT (safegoto SOURCES safegoto.cpp LINKFLAGS)
target_link_libraries(mapper ${GLUT_LIBRARY} ${OPENGL_LIBRARY} X11 Xext)
