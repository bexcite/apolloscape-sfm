# Get the project_name from the current directory
get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
cmake_minimum_required(VERSION 2.8.11)

project(${project_name})

# set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -Wno-unused-variable -pedantic -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-g -00")
set(CMAKE_CXX_FLAGS_RELEASE "-03")

# include_directories(${PROJECT_SOURCE_DIR})

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
# find_package(Boost 1.59 REQUIRED COMPONENTS filesystem)
# find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(OpenCV REQUIRED)

message("Project Name =======: " ${PROJECT_NAME})

# glad
set(GLAD_DIR ${PROJECT_SOURCE_DIR}/lib/glad)
set(GLAD_INCLUDE_DIRS ${GLAD_DIR}/include)
add_library(glad ${GLAD_DIR}/src/glad.c)
target_include_directories(glad PRIVATE ${GLAD_INCLUDE_DIRS})

# enable_testing()

set(PROJECT_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)


set(GLFW_DIR ${PROJECT_SOURCE_DIR}/lib/glfw-3.2.1)
# set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
# set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
# set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
# set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")
add_subdirectory(${GLFW_DIR})


add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)




# add_executable(${project_name} ${project_name}.cpp utils.cpp)
# target_include_directories(${project_name} PUBLIC
#     ${Boost_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
#   )
# target_link_libraries(${project_name} ${OpenCV_LIBS} ${Boost_LIBRARIES})

# message(STATUS "Target=====: ${CMAKE_BUILD_TYPE}")

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
