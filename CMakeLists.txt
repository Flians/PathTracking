cmake_minimum_required(VERSION 3.0.0)
project(PathTracking VERSION 0.1.0)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O0 -march=core2 -Wno-ignored-attributes")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O2 -march=core2 -Wno-ignored-attributes")
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories()
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
    ./PID
    ./LEQ
    ./Spline
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

file(GLOB SOURCES
    "*.cpp"
    "*.cxx"
    "./PID/*.cpp"
    "./LEQ/*.cpp"
    "./Spline/*.cpp"
    )

set( PROJECT_LINK_LIBS
    ${OpenCV_LIBRARIES}
     glog
)

add_executable(PathTracking ${SOURCES})
target_link_libraries(PathTracking ${PROJECT_LINK_LIBS})

add_library(libPathTracking ${SOURCES})


