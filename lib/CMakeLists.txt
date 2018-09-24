set(STB_DIR ${PROJECT_SOURCE_DIR}/lib/stb)
add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE ${STB_DIR}/include )

set(GLM_DIR ${PROJECT_SOURCE_DIR}/lib/glm-0.9.9.2)
message("stb GLM_DIR = " ${GLM_DIR})

set(GLM_TEST_ENABLE OFF)
add_subdirectory(${GLM_DIR})
