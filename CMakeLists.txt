# Get the project_name from the current directory
cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)

get_filename_component(project_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

project(${project_name} VERSION 0.0.1)

set(CMAKE_BUILD_TYPE Debug)

# set(CMAKE_CXX_STANDARD 11)
# add_compile_options(-Wall)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -Wno-unused-variable -pedantic -Werror")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -00")
# set(CMAKE_CXX_FLAGS_RELEASE "-03")

# include_directories(${PROJECT_SOURCE_DIR}) 

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.59 REQUIRED COMPONENTS filesystem)
# find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

message("OpenCV Version =======: " ${OpenCV_VERSION})

message("Project Name =======: " ${PROJECT_NAME})

# glad
set(GLAD_DIR ${PROJECT_SOURCE_DIR}/lib/glad)
set(GLAD_INCLUDE_DIRS ${GLAD_DIR}/include)
add_library(glad ${GLAD_DIR}/src/glad.c)
target_include_directories(glad PUBLIC ${GLAD_INCLUDE_DIRS})
target_link_libraries(glad)
# target_link_libraries(glad PRIVATE ${CMAKE_DL_LIBS})

# enable_testing()

set(PROJECT_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)


set(GLFW_DIR ${PROJECT_SOURCE_DIR}/lib/glfw-3.2.1)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
set(GLFW_BUILD_DOCS OFF)
set(GLFW_INSTALL OFF)
message("GLFW_BUILD_TESTS =======: " ${GLFW_BUILD_TESTS})
add_subdirectory(${GLFW_DIR})

# add libs
add_subdirectory(${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

add_subdirectory(${PROJECT_SOURCE_DIR}/tests)




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
