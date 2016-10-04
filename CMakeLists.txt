cmake_minimum_required(VERSION 2.8.3)
project(dbscan)

find_package(catkin_simple 0.1.0 REQUIRED )

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME} src/dbscan.cc)

#add_doxygen(NOT_AUTOMATIC)

cs_install()
cs_export()

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
