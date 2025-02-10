add_library(stb OBJECT stb_impl.c)
target_include_directories(stb PUBLIC ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(stb PROPERTIES
    CXX_STANDARD 17
)
