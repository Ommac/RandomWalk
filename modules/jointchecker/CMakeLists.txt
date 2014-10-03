# Copyright: 2014 iCub Facility, Istituto Italiano di Tecnologia
# Author: Raffaello Camoriano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME jointchecker)
PROJECT(${PROJECTNAME})

file(GLOB source src/*.cpp)
#file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
#source_group("Header Files" FILES ${header})

IF(NOT ICUB_USE_IPOPT)
    MESSAGE(FATAL_ERROR "IPOPT is required")
ENDIF(NOT ICUB_USE_IPOPT)

SET(folder_source main.cpp)
SOURCE_GROUP("Source Files" FILES ${folder_source})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")


include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${PROJECTNAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
