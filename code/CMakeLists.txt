
# cmake file created following instructions at
# https://cmu-perceptual-computing-lab.github.io/openpose/web/html/doc/md_doc_advanced_deployment.html and
# https://www.yarp.it/git-master/yarp_cmake_hello.html


#get_filename_component(SOURCE_NAME test.cpp NAME_WE)
#
#if (UNIX OR APPLE)
#  set(EXE_NAME "${SOURCE_NAME}.bin")
#elseif (WIN32)
#  set(EXE_NAME "test")
#endif ()
#
#add_executable(${EXE_NAME} test.cpp)
#target_link_libraries(${EXE_NAME} openpose ${examples_3rdparty_libraries})
#
#if (WIN32)
#  set_property(TARGET ${EXE_NAME} PROPERTY FOLDER "Test")
#  configure_file(${CMAKE_SOURCE_DIR}/cmake/OpenPose${VCXPROJ_FILE_GPU_MODE}.vcxproj.user
#      ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME}.vcxproj.user @ONLY)
#  # Properties->General->Output Directory
#  set_property(TARGET ${EXE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/$(Platform)/$(Configuration))
#  set_property(TARGET ${EXE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/$(Platform)/$(Configuration))
#endif (WIN32)


cmake_minimum_required(VERSION 3.16.0)
 
project(OpenPoseYarpTest VERSION 0.1)

#add_definitions(-std=c++11)
 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
 
find_package(GFlags)
find_package(Glog)
find_package(OpenCV)
find_package(OpenPose REQUIRED)
find_package(YARP COMPONENTS cv os sig REQUIRED)
 
include_directories(${OpenPose_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIR} ${GLOG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_executable(test.bin test.cpp)
 
target_link_libraries(test.bin ${OpenPose_LIBS} ${GFLAGS_LIBRARY} ${GLOG_LIBRARY} ${OpenCV_LIBS} YARP::YARP_cv YARP::YARP_os YARP::YARP_sig YARP::YARP_init)
