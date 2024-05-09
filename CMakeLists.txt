cmake_minimum_required(VERSION 3.25.2)

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
# if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
#     set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")
# endif()

add_subdirectory(external/stb)
add_subdirectory(Ds)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(cgspline
    src/BezierSurface.cpp
    src/GLBezierSurface.cpp
    src/Framebuffer.cpp
    src/Log.cpp
    src/Main.cpp
    src/MainWindow.cpp
    src/MainWindow_UI.cpp
    src/PatchBVH.cpp
    src/RayTracer.cpp
    src/SceneReaderExt.cpp
    src/Spline_BezierClipping.cu
    src/Spline_Subdivision.cpp
    src/Spline.cu
    src/SplRenderer.cpp
    src/Surface.cpp
    src/Texture.cpp
    src/rt/BVH.cpp
    src/rt/CPURayTracer.cpp
    src/rt/CUDAUtility.cpp
    src/rt/PBR.cu
    src/rt/PerfectHashFunction.cu
    src/rt/Primitive.cu
    src/rt/RayTracer.cu
    src/rt/Scene.cu
    src/rt/SoA.cpp
)

target_compile_options(cgspline PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
    -lineinfo -rdc=true -maxrregcount=128
    --generate-code arch=compute_75,code=lto_75
    -Xptxas=-v -Xnvlink=-dlto
>)

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

if(NOT ${CMAKE_BUILD_TYPE} MATCHES "Release")
    target_compile_definitions(cgspline PRIVATE CG_GL_DEBUG)
endif()

target_link_libraries(cgspline cgcuda stb)
