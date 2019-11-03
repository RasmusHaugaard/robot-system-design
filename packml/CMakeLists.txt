cmake_minimum_required(VERSION 2.8.3)

project(packML LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
#endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

add_executable(packml
    mainwindow.ui
    mainwindow.cpp
    main.cpp
    mystate.cpp
    mystate.h
    listener.cpp
    listener.h
)
target_link_libraries(packml Qt5::Widgets ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
