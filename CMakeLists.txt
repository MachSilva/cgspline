cmake_minimum_required(VERSION 3.16)

project(cgspline)

add_subdirectory(Ds)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

if(UNIX)
    set(CMAKE_EXECUTABLE_SUFFIX ".out")
endif()

add_executable(cgspline
    src/BezierPatches.cpp
    src/Main.cpp
    src/MainWindow.cpp
    src/PatchBVH.cpp
    src/RayTracer.cpp
    src/Spline.cpp
    src/Surface.cpp
)

target_link_libraries(cgspline cg)
