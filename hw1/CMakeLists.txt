cmake_minimum_required(VERSION 2.4)
PROJECT (MRP_homework)

SET (CMAKE_MODULE_PATH "~/Documents/School/20113/mrp/cmake/")
INCLUDE (UsePlayerC++)

PLAYER_ADD_PLAYERCPP_CLIENT (goto SOURCES goto.cpp LINKFLAGS)
