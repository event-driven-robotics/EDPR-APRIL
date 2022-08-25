# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(april_nchpe)

if(NOT CMAKE_BUILD_TYPE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

#include(GNUInstallDirs)

# mandatory use of these packages
find_package(MetavisionSDK COMPONENTS core driver QUIET)
find_package(OpenCV REQUIRED)
find_package(hpe-core REQUIRED)

#include(AddInstallRPATHSupport)
#add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
#                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
#                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
#                          USE_LINK_PATH)

add_executable(${PROJECT_NAME} eventhpe.cpp erosdirect.h)
#target_compile_options(sixdofobject PRIVATE -Werror -Wall -Wextra)
target_link_libraries(${PROJECT_NAME} PRIVATE  hpe-core::hpe-core
                                            
                                            
                                            ${OpenCV_LIBS}
                                            MetavisionSDK::core
                                            MetavisionSDK::driver
                                            )

#install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

