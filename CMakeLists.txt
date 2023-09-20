cmake_minimum_required(VERSION 3.10)
project(Registeration)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(proxsuite REQUIRED)


include_directories("spectra/1.0.1/include")
include_directories("eigen-3.4.0")
include_directories("Meshlib")
include_directories("basic_opengl")
include_directories("TMeshlib")
include_directories("freeglut")


add_executable(Registeration main.cpp Getpoint.h CGetPoint.h MeshBuilder.h Tool.h ToolMesh.h CGetpoint.cpp Registeration.h Registeration.cpp)

target_link_libraries(Registeration PUBLIC proxsuite::proxsuite-vectorized)
