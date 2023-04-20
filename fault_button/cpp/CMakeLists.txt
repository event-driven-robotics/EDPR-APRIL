# requires minimum cmake version
cmake_minimum_required(VERSION 3.16.0)

# produce the cmake var PROJECT_NAME
project(visual-fault-button)

find_package(YARP COMPONENTS os REQUIRED)
find_package(event-driven REQUIRED)

# final isaac application
add_executable(${PROJECT_NAME} vfb.cpp)
target_link_libraries(${PROJECT_NAME} YARP::YARP_OS 
                                      ev::event-driven)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})