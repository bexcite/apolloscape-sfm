message("my INCLUDE DIRS = " ${PROJECT_INCLUDE_DIRS})

add_executable(${project_name} main.cpp)
target_include_directories(${project_name} PUBLIC
    ${PROJECT_INCLUDE_DIRS}
  )
target_link_libraries(${project_name} glfw)

set_target_properties(${project_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
