cmake_minimum_required(VERSION 2.8)
project(pcl2grid)
set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 3.0 QUIET)
include_directories(${OpenCV_INCLUDE_DIR})
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Build examples

INCLUDE_DIRECTORIES(
                        ${PROJECT_SOURCE_DIR}/
                        ${PROJECT_SOURCE_DIR}/header
                    )
add_compile_options(-w -g) #-fno-elide-constructors)
add_executable(remove_outlier src/remove_outlier.cc src/heatmap_filtering.cpp)
target_link_libraries(remove_outlier ${OpenCV_LIBRARIES}
${PCL_LIBRARIES})


