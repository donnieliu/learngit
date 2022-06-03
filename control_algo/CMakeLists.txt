cmake_minimum_required(VERSION 3.5)
project(control_algo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#把对应目录下的c和c++所有源文件分别保存在变量中

file(GLOB_RECURSE  SOURCE_SRC src/*.cc)
file(GLOB_RECURSE  CONTROLLER_SRC 
  controller/lqr_acc/*.cpp) 

file(GLOB_RECURSE  SOURCE_SRC src/*.cpp)
file(GLOB_RECURSE  CONTROLLER_SRC
  controller/lqr_acc/*.cpp
  controller/purepursuit/*.cpp
  controller/EPSAngle2Torque_PID/*.cpp
  )
file(GLOB_RECURSE  COMMON_SRC
  common/acc_params/*.cpp
  common/ego_state/*.cpp
  common/mathtool/*.cpp
  common/vehicle_params/*.cpp)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)

find_package(x_buf_api REQUIRED)
find_package(icvos_interface REQUIRED)
find_package(msgs_pack REQUIRED)

include_directories(
  include
  common
  controller
  controller/EPSAngle2Torque_PID
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${x_buf_api_INCLUDE_DIRS}
  ${icvos_interface_INCLUDE_DIRS}
  ${msgs_pack_INCLUDE_DIRS}
)
link_libraries(
    ${x_buf_api_LIBRARIES}
    ${icvos_interface_LIBRARIES}
)

add_library(${PROJECT_NAME} SHARED
    src/control_algo.cc
    interface/control_tt.cc
    ${SOURCE_SRC}
    ${CONTROLLER_SRC}
    ${COMMON_SRC}
    ${MATH_SRC}
)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    glog
    yaml-cpp
)

install(
    TARGETS control_algo
    DESTINATION lib
)
install(
    DIRECTORY conf/
    DESTINATION conf
)

ament_package()

