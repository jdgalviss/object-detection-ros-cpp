cmake_minimum_required(VERSION 3.10.1)

set(CMAKE_CXX_STANDARD 14)
project(objectdetector)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14 -pthread")

find_package(OpenCV 4.1 REQUIRED )
# include_directories([BEFORE] "/usr/local/include/opencv4")

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Find all executables
file(GLOB SOURCES "src/*.cpp") #src/*.h

add_executable(objectdetector ${SOURCES})
target_link_libraries(objectdetector ${OpenCV_LIBS} )