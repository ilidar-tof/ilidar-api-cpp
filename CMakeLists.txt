# This example tested on cmake V2.8+
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Project name
project(ilidar_api_cpp_cmake)

# Set cmake build type to Release
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set API files
set(API_PATH "src/")
FILE(GLOB API_SRC
  "${API_PATH}/*.hpp"
  "${API_PATH}/*.cpp"
)

# Add build targets
add_executable(helloworld_cmake helloworld/helloworld.cpp ${API_SRC})
add_executable(opencv_example_cmake opencv/opencv_example.cpp ${API_SRC})
add_executable(pcl_example_cmake pcl/pcl_example.cpp ${API_SRC})
add_executable(streamer_cmake streamer/streamer.cpp ${API_SRC})
add_library(ilidar SHARED dll/dllexport.cpp ${API_SRC})


# Use OpenCV 4.0+
if (WIN32)
  set(OpenCV_DIR "C:/opencv/build/x64/vc15/lib")
else ()
  set(CMAKE_THREAD_LIBS_INIT "-lpthread")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  set(CMAKE_HAVE_THREADS_LIBRARY 1)
  set(CMAKE_USE_WIN32_THREADS_INIT 0)
  set(CMAKE_USE_PTHREADS_INIT 1)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
endif ()

find_package(OpenCV 4.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
target_link_libraries(opencv_example_cmake ${OpenCV_LIBS})

# Use PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (pcl_example_cmake ${PCL_LIBRARIES})
