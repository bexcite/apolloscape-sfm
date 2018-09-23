set(STB_DIR ${PROJECT_SOURCE_DIR}/lib/stb)

message("stb INCLUDE DIR = " ${STB_DIR}/include)

add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE ${STB_DIR}/include )
