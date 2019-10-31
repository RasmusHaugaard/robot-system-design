cmake_minimum_required(VERSION 3.5)
project(light_tower)

find_package(ur_rtde REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(light_tower main.cpp)
target_link_libraries(light_tower PRIVATE ur_rtde::rtde ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
