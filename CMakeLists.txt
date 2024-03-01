cmake_minimum_required(VERSION 3.18)

project(cgspline VERSION 1.0 LANGUAGES C CXX CUDA)

if(UNIX)
    set(CMAKE_EXECUTABLE_SUFFIX ".out")
    # if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    #     add_compile_options(-fsanitize=address -g3 -fno-fast-math)
    #     add_link_options(-fsanitize=address)
    #     message(STATUS "Address sanitizer enabled")
    # endif()
endif()

set(USE_CUDA ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_ARCHITECTURES 75)

add_subdirectory(external/stb)
add_subdirectory(Ds)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(cgspline
    src/BezierPatches.cpp
    src/Framebuffer.cpp
    src/Main.cpp
    src/MainWindow.cpp
    src/PatchBVH.cpp
    src/RayTracer.cpp
    src/Spline.cpp
    src/Spline_Subdivision.cpp
    src/Spline_BezierClipping.cpp
    src/Surface.cpp
    src/Texture.cpp

    src/SceneReaderExt.cpp
)

get_target_property(cgdemo_src cgdemo SOURCE_DIR)

target_sources(cgspline PRIVATE
    ${cgdemo_src}/reader/AbstractParser.cpp
    ${cgdemo_src}/reader/Buffer.cpp
    ${cgdemo_src}/reader/ErrorHandler.cpp
    ${cgdemo_src}/reader/Expression.cpp
    ${cgdemo_src}/reader/FileBuffer.cpp
    ${cgdemo_src}/reader/ReaderBase.cpp
    ${cgdemo_src}/reader/SceneReader.cpp
    ${cgdemo_src}/reader/Scope.cpp
)

target_include_directories(cgspline PRIVATE ${cgdemo_src})

# target_compile_definitions(cgspline PRIVATE SPL_BC_STATS)

if(NOT ${CMAKE_BUILD_TYPE} MATCHES "Release")
    target_compile_definitions(cgspline PRIVATE CG_GL_DEBUG)
endif()

target_link_libraries(cgspline cgcuda stb)
