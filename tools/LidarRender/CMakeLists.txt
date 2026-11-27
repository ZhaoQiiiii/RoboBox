cmake_minimum_required(VERSION 3.16)
project(lidar_render C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LIDAR_RENDER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/marsim_render/" CACHE STRING "marsim_render ROOT_DIR")
add_definitions(-DROOT_DIR="${LIDAR_RENDER_ROOT_DIR}")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(glad STATIC marsim_render/include/glad/glad.c)
target_include_directories(glad PUBLIC marsim_render/include)
target_link_libraries(glad PUBLIC ${CMAKE_DL_LIBS})
add_library(marsim_render STATIC marsim_render/src/marsim_render.cpp)
target_include_directories(marsim_render PUBLIC marsim_render/include)
target_link_libraries(marsim_render PUBLIC
  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} glfw OpenGL::GL
  yaml-cpp OpenMP::OpenMP_CXX glad Eigen3::Eigen)

add_library(protocol STATIC src/protocol.cpp)
target_include_directories(protocol PUBLIC src)
target_link_libraries(protocol PUBLIC Eigen3::Eigen jsoncpp_lib)

add_executable(render_once tests/render_once.cpp)
target_link_libraries(render_once PRIVATE marsim_render)

enable_testing()
add_executable(protocol_test tests/protocol_test.cpp)
target_link_libraries(protocol_test PRIVATE protocol)
add_test(NAME protocol_test COMMAND protocol_test)

add_executable(lidar_render_tcp src/lidar_render_tcp.cpp)
target_link_libraries(lidar_render_tcp PRIVATE protocol marsim_render)
