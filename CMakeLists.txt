cmake_minimum_required(VERSION 3.16.3)
project(iiwa_ros)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Armadillo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs roscpp)

if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(EXECUTABLE_OUTPUT_PATH build/)
include_directories(${ARMADILLO_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OpenMP_INCLUDE_DIRS} "/home/zxl5344/IROS2022/src/iiwa_ros/iiwa_ros/lus-rosscripts")
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH build/)

add_executable(errsac_cp errsac_cartpend.cpp)
target_link_libraries(errsac_cp ${ARMADILLO_LIBRARIES})

add_executable(figure8 figure8.cpp)
target_link_libraries(figure8 
	${ARMADILLO_LIBRARIES}
	${catkin_LIBRARIES}
	${std_msgs_LIBRARIES})


