cmake_minimum_required(VERSION 3.21)

project(cgspline)

if(UNIX)
    set(CMAKE_EXECUTABLE_SUFFIX ".out")
    # if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    #     add_compile_options(-fsanitize=address -g3 -fno-fast-math)
    #     add_link_options(-fsanitize=address)
    #     message(STATUS "Address sanitizer enabled")
    # endif()
endif()

add_subdirectory(external/stb)
add_subdirectory(Ds)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
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

target_sources(cgspline PRIVATE
    ${cgdemo_SOURCE_DIR}/reader/AbstractParser.cpp
    ${cgdemo_SOURCE_DIR}/reader/Buffer.cpp
    ${cgdemo_SOURCE_DIR}/reader/ErrorHandler.cpp
    ${cgdemo_SOURCE_DIR}/reader/Expression.cpp
    ${cgdemo_SOURCE_DIR}/reader/FileBuffer.cpp
    ${cgdemo_SOURCE_DIR}/reader/ReaderBase.cpp
    ${cgdemo_SOURCE_DIR}/reader/SceneReader.cpp
    ${cgdemo_SOURCE_DIR}/reader/Scope.cpp
)
target_include_directories(cgspline PRIVATE ${cgdemo_SOURCE_DIR})

# target_compile_definitions(cgspline PRIVATE SPL_BC_STATS)

if(NOT ${CMAKE_BUILD_TYPE} MATCHES "Release")
    target_compile_definitions(cgspline PRIVATE CG_GL_DEBUG)
endif()

target_link_libraries(cgspline cg stb)
