cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-O2")
#set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        fsd_common_msgs
        fsd_tools
	fssim_common
        )

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen3 QUIET)


find_package(Python COMPONENTS Development)
#find_package(IPOPT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)





catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES hello
        CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs
        #   DEPENDS ipopt
 	DEPENDS ${PYTHON_LIBRARIES}
)

#set(libgp_DIR "/home/shuangji/libgp")


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${IPOPT_INCLUDE_DIRS}
        /usr/include/eigen3
	#${libgp_DIR}/include
)

add_executable(${PROJECT_NAME}
        src/control.cpp
        src/control_handle.cpp
        src/main.cpp
        include/Utils/types.h
        src/Solver/solver_base.cpp
        src/Solver/mpc_kin_solver.cpp
      #  src/Solver/mpc_mine.cpp
      #  src/Solver/mpc_osqp_solver.cpp
        src/Solver/stanley_solver.cpp
        src/Solver/lqr_solver.cpp
        src/Solver/pid_controller.cpp
       # src/Solver/mpc_lat.cpp
        src/Utils/visual.cpp
     )
#link_directories(${libgp_DIR}/lib)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${IPOPT_LIBRARIES}
        ${OpenCV_LIBS}
 	#libosqp.so  
 	#libOsqpEigen.so 
 	#libqdldl.so 
 	#osqp
 	#OsqpEigen
         m
         ipopt
	#${libgp_DIR}/build/libgp.a
 	${PYTHON_LIBRARIES}
	#casadi
        )
target_include_directories(${PROJECT_NAME} PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Python_LIBRARIES})

message(STATUS "IPOPT LIBRARIES:" ${IPOPT_LIBRARIES})
