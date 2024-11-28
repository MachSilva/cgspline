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

add_subdirectory(external/stb)
add_subdirectory(Ds)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(cgspline
    src/GLSurface.cpp
    src/Surface.cpp
    src/SurfacePipeline.cpp
    src/Framebuffer.cpp
    src/Log.cpp
    src/Main.cpp
    src/MainWindow.cpp
    src/MainWindow_UI.cpp
    src/PatchBVH.cpp
    src/RayTracer.cpp
    # src/SceneReaderExt.cpp
    src/Spline_BezierClipping.cu
    src/Spline_Subdivision.cpp
    src/Spline.cu
    src/SplRenderer.cpp
    src/Texture.cpp
    src/reader/DescriptionLanguage.cpp
    src/reader/Parser.cpp
    src/reader/SceneReader.cpp
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

# target_compile_definitions(cgspline PRIVATE SPL_BC_STATS)

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    target_compile_options(cgspline PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
        -g -G
        --generate-code arch=compute_75,code=sm_75
        -Xptxas=-v
    >)
elseif(${CMAKE_BUILD_TYPE} MATCHES "RelWithDebInfo")
    target_compile_options(cgspline PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
        -G -dopt=on -maxrregcount=168
        --generate-code arch=compute_75,code=sm_75
        -Xptxas=-v
    >)
else()
    target_compile_options(cgspline PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
        -lineinfo -rdc=true -maxrregcount=168
        --generate-code arch=compute_75,code=lto_75
        -Xptxas=-v -Xnvlink=-dlto
    >)
endif()

if(NOT ${CMAKE_BUILD_TYPE} MATCHES "Release")
    target_compile_definitions(cgspline PRIVATE CG_GL_DEBUG)
endif()

target_link_libraries(cgspline cgcuda stb)

target_precompile_headers(cgspline PRIVATE
    <imgui.h>
    <imgui_internal.h>
    <stb_image.h>
    <cuda_runtime.h>
    <math/Vector4.h>
    <math/Matrix4x4.h>
    <geometry/Bounds3.h>
    <graph/SceneWindow.h>
)
