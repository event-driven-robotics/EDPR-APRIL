# requires minimum cmake version
cmake_minimum_required(VERSION 3.16.0)

# produce the cmake var PROJECT_NAME
project(edpr-april)

find_package(YARP COMPONENTS os sig cv REQUIRED)
find_package(hpe-core REQUIRED)
find_package(event-driven REQUIRED)

# final isaac application
add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} YARP::YARP_OS 
                                      YARP::YARP_sig
                                      YARP::YARP_cv
                                      ev::event-driven
                                      hpe-core::hpe-core)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})