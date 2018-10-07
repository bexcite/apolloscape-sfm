message("my INCLUDE DIRS = " ${PROJECT_INCLUDE_DIRS})

# ==================================
# cv_gl_lib - library with all shared code
add_library(cv_gl_lib shader.cpp camera.cpp mesh.cpp gl_window.cpp)
set_property(TARGET cv_gl_lib PROPERTY CXX_STANDARD 11)
target_include_directories(cv_gl_lib PUBLIC ${PROJECT_INCLUDE_DIRS} )
target_link_libraries(cv_gl_lib PUBLIC glad glfw stb_image glm assimp)
set_target_properties(cv_gl_lib PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


add_executable(${PROJECT_NAME} main.cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
# target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} PUBLIC cv_gl_lib)
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


set(EDIT_SPACE_NAME edit_space)
add_executable(${EDIT_SPACE_NAME} edit_space.cpp)
set_property(TARGET ${EDIT_SPACE_NAME} PROPERTY CXX_STANDARD 11)
message("edit_space_name = " ${EDIT_SPACE_NAME})
target_link_libraries(${EDIT_SPACE_NAME} PUBLIC cv_gl_lib)
set_target_properties(${EDIT_SPACE_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# Add GLAD
# target_include_directories(${PROJECT_NAME} PRIVATE ${GLAD_DIR}/include)
# target_link_libraries(${PROJECT_NAME} glad ${CMAKE_DL_LIBS})
# target_link_libraries(${PROJECT_NAME} PRIVATE glad)

# Add GLFW
# target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
# target_link_libraries(${PROJECT_NAME} glfw ${GLFW_LIBRARIES})
# target_include_directories(${PROJECT_NAME} PRIVATE ${GLFW_DIR}/include)






# Add stb_image
# target_link_libraries(${PROJECT_NAME} PRIVATE stb_image)

# target_link_libraries(${PROJECT_NAME} PRIVATE glm)

# set(ASSIMP_DIR ${PROJECT_SOURCE_DIR}/lib/assimp-4.1.0)
# target_link_libraries(${PROJECT_NAME} PRIVATE assimp)
# target_include_directories(${PROJECT_NAME} PUBLIC ${ASSIMP_DIR}/include )
# message("===== ASSIMP INCLUDE DIR" ${ASSIMP_INCLUDE_DIRS})
