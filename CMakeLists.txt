cmake_minimum_required(VERSION 3.20)
project(run)

set(CMAKE_CXX_STANDARD 11)
set(CAM_LIB ${PROJECT_SOURCE_DIR}/lib/MVCAMSDK.lib)
set(CAM_LIB64 ${PROJECT_SOURCE_DIR}/lib/MVCAMSDK_X64.lib)


find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include)


aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRC)

add_executable(run main_debug.cpp)

target_link_libraries(run SRCSO ${OpenCV_LIBS} ${CAM_LIB} ${CAM_LIB64})

add_library(SRCSO ${DIR_SRC})