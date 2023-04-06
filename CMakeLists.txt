cmake_minimum_required(VERSION 3.10)

project(ColmapProject CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# opencv
FIND_PACKAGE(OpenCV REQUIRED)
if (OPENCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif (OPENCV_FOUND)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 多线程
# find_package(Threads)

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include )

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_DIR)

SET(EXTRA_LIBS ${EXTRA_LIBS} ${OpenCV_LIBS})


file(GLOB example_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp)
message("exam_files ${example_files}")
foreach(example_file ${example_files})

    get_filename_component(example_name ${example_file} NAME_WE)
    add_executable(${example_name} ${example_file} ${SRC_DIR})
    target_link_libraries(${example_name} ${EXTRA_LIBS} -lpthread -ldl Sophus::Sophus)

endforeach(example_file ${example_files})