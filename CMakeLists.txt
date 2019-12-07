cmake_minimum_required(VERSION 3.15)
project(hamiltonian_msckf)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(src)

add_library(right_msckf
        src/hamiltonian_msckf.h
        src/hamiltonian_msckf.cpp
        src/vio_simulator.h
        src/vio_simulator.cpp
        src/b3_spline.h
        src/b3_spline.cpp
        src/core.cpp src/core.h)



add_executable(main executables/main.cpp)

target_link_libraries(main right_msckf)
