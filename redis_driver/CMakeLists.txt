cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(sai_franka_robot_redis_driver)

# CMAKE OPTIONS
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
# SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# BUILD SOURCES AND LIBRARY
set(SOURCES 
    main.cpp
)

# Create a library
add_executable(${PROJECT_NAME} ${SOURCES})
add_executable(sai_franka_gripper_redis_driver gripper.cpp)

# LIBFRANKA
find_package(Franka REQUIRED PATHS ${PROJECT_SOURCE_DIR}/../libfranka/build)
set(Franka_INCLUDE_DIR ${Franka_DIR}/../include)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
    find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP jsoncpp)
    include_directories(${JSONCPP_INCLUDE_DIRS})
    link_directories(${JSONCPP_LIBRARY_DIRS})
    set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

# - tinyxml2
find_library(TINYXML_LIBRARIES tinyxml2)

# - eigen3
find_package(Eigen3 REQUIRED)

# create headers list
set(franka_panda_redis_driver_INCLUDE_DIRS 
    ${Franka_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIR}
)

# create libraries list
set(franka_panda_redis_driver_LIBRARIES 
    ${Franka_LIBRARIES}   
    ${HIREDIS_LIBRARY}
    ${JSONCPP_LIBRARY}
    ${TINYXML_LIBRARIES}
    -lrt
    -lpthread
)

include_directories( ${franka_panda_redis_driver_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${franka_panda_redis_driver_LIBRARIES})
target_link_libraries(sai_franka_gripper_redis_driver ${franka_panda_redis_driver_LIBRARIES})

set(CONFIG_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/config_folder")
add_definitions(-DCONFIG_FOLDER="${CONFIG_FOLDER}")