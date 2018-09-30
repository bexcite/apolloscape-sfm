set(STB_DIR ${PROJECT_SOURCE_DIR}/lib/stb)
add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE ${STB_DIR}/include )

set(GLM_DIR ${PROJECT_SOURCE_DIR}/lib/glm-0.9.9.2)
message("stb GLM_DIR = " ${GLM_DIR})

set(GLM_TEST_ENABLE OFF)
add_subdirectory(${GLM_DIR})

# set(ASSIMP_DIR ${PROJECT_SOURCE_DIR}/lib/assimp-4.1.0)
set(ASSIMP_DIR ${PROJECT_SOURCE_DIR}/lib/assimp-20180928)
option(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
option(ASSIMP_BUILD_SAMPLES OFF)
option(ASSIMP_BUILD_TESTS OFF)
option(BUILD_DOCS ON)
add_subdirectory(${ASSIMP_DIR})
