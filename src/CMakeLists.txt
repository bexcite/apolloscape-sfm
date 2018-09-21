message("my INCLUDE DIRS = " ${PROJECT_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_INCLUDE_DIRS} )

# Add GLAD
# target_include_directories(${PROJECT_NAME} PRIVATE ${GLAD_DIR}/include)
# target_link_libraries(${PROJECT_NAME} glad ${CMAKE_DL_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE glad)

# Add GLFW
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
# target_link_libraries(${PROJECT_NAME} glfw ${GLFW_LIBRARIES})
# target_include_directories(${PROJECT_NAME} PRIVATE ${GLFW_DIR}/include)


# target_compile_definitions(${PROJECT_NAME} PRIVATE GLFW_INCLUDE_NONE)


# target_link_libraries(${project_name} glfw)

set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
