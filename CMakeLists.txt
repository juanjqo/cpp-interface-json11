CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(dqrobotics-interface-json11)
set (CMAKE_CXX_STANDARD 11)

#ADD_DEFINITIONS(-g -O2 -Wall)
FIND_PACKAGE(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(EIGEN3_INCLUDE_DIR)
INCLUDE_DIRECTORIES(${PROJECT_NAME} include)
INCLUDE_DIRECTORIES(${PROJECT_NAME} dropbox/json11)
INCLUDE_DIRECTORIES(${PROJECT_NAME} cpp/include)

################################################################
# DEFINE AND INSTALL LIBRARY AND INCLUDE FOLDER
################################################################

#add_executable(${PROJECT_NAME}
#    src/dqrobotics/interfaces/DQ_JsonReader.cpp
#
#    cpp/src/DQ.cpp
#    cpp/src/robot_modeling/DQ_Kinematics.cpp
#    cpp/src/robot_modeling/DQ_SerialManipulator.cpp
#
#    src/dqrobotics/interfaces/DQ_JsonReader.cpp
#    dropbox/json11/json11.cpp
#    )

ADD_LIBRARY(${PROJECT_NAME} SHARED
    src/dqrobotics/interfaces/json11/DQ_JsonReader.cpp
    dropbox/json11/json11.cpp
    )

SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    include/dqrobotics/interfaces/json11/DQ_JsonReader.h
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces/json11"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES
    src/dqrobotics/interfaces/json11/DQ_JsonReader.cpp
    DESTINATION "src/dqrobotics/interfaces/json11")
