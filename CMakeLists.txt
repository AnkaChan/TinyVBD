cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

project(TinyVBD LANGUAGES CXX)

## Use C++11
set (CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB SRC
    "*.h"
    "*.cpp"
	"*.c"
	"*.cu"
	)

add_executable(P16_VBDWithConstraint 
	${SRC}
	${EBD_SRCS}
)

