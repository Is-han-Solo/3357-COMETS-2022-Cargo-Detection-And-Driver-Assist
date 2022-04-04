cmake_minimum_required(VERSION 3.10)

project(ball_vision)

add_executable(ball_vision main.cpp ImageProcessing.cpp)

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)

find_package(Threads REQUIRED)

find_package(wpilib REQUIRED)

include_directories(
	
	${OpenCV_INCLUDE_DIRS}


)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_include_directories(
	ball_vision PUBLIC 
	
	#../../ntcore/include/

	#../../ntcore/wpiutil/include/
)

target_link_libraries(
	ball_vision PUBLIC 

	${OpenCV_LIBS} 
	
	${realsense2_LIBRARY}

	cameraserver

	cscore

	ntcore

	wpiutil

	#Threads::Threads 

	#${CMAKE_SOURCE_DIR}/../../ntcore/libntcore.so

	#${CMAKE_SOURCE_DIR}/../../ntcore/libwpiutil.a
)
