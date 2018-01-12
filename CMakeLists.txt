cmake_minimum_required(VERSION 3.9)
project(calibrateCameraCPP)

set(OpenCV_DIR /home/alexey/work/programming/opencv/build)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp)

add_executable(calibrateCameraCPP main.cpp)
target_link_libraries(calibrateCameraCPP ${OpenCV_LIBS})

#************* BOOST ************************

set (Boost_USE_STATIC_LIBS ON)
set (Boost_USE_MULTITHREADED ON)

set (BOOST_COMPONENTS
        thread
        filesystem)

find_package (Threads)

find_package (Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries (calibrateCameraCPP ${Boost_LIBRARIES})

