cmake_minimum_required(VERSION 3.16)

project(cgspline)

add_subdirectory(Ds)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(cgspline
    src/Surface.cpp
    src/BezierPatches.cpp
    src/Main.cpp
    src/MainWindow.cpp
)

target_link_libraries(cgspline cg)

# Symlink assets directory
execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
        ${CMAKE_CURRENT_LIST_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
)
