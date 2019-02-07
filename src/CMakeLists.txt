message("my INCLUDE DIRS = " ${PROJECT_INCLUDE_DIRS})

# ==================================
# cv_gl_lib - library with all shared code //  sfm.cpp
add_library(cv_gl_lib shader.cpp camera.cpp mesh.cpp gl_window.cpp utils.cpp sfm.cpp sfm_common.cpp)
set_property(TARGET cv_gl_lib PROPERTY CXX_STANDARD 11)
target_include_directories(cv_gl_lib PUBLIC ${PROJECT_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
# message("boost INCLUDE_DIRS = " ${Boost_INCLUDE_DIRS})
# message("boost LIBRARIES = " ${Boost_LIBRARIES})
target_link_libraries(cv_gl_lib PUBLIC 
    glad 
    glfw 
    stb_image 
    glm assimp 
    Boost::filesystem 
    opencv_imgcodecs 
    opencv_calib3d 
    opencv_xfeatures2d
    ${CERES_LIBRARIES}
    cereal
    )
set_target_properties(cv_gl_lib PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


#[[
]]
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


set(DEBUG_CUBE_NAME debug_cube)
add_executable(${DEBUG_CUBE_NAME} debug_cube.cpp)
set_property(TARGET ${DEBUG_CUBE_NAME} PROPERTY CXX_STANDARD 11)
message("debug_cube_name = " ${DEBUG_CUBE_NAME})
target_link_libraries(${DEBUG_CUBE_NAME} PUBLIC cv_gl_lib)
set_target_properties(${DEBUG_CUBE_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


set(SHOW_CAMERAS_NAME show_cameras)
add_executable(${SHOW_CAMERAS_NAME} show_cameras.cpp)
set_property(TARGET ${SHOW_CAMERAS_NAME} PROPERTY CXX_STANDARD 11)
message("show_cameras_name = " ${SHOW_CAMERAS_NAME})
target_link_libraries(${SHOW_CAMERAS_NAME} PUBLIC cv_gl_lib)
set_target_properties(${SHOW_CAMERAS_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


set(TWO_CAMERAS_NAME two_cameras)
add_executable(${TWO_CAMERAS_NAME} two_cameras.cpp)
set_property(TARGET ${TWO_CAMERAS_NAME} PROPERTY CXX_STANDARD 11)
message("two_cameras_name = " ${TWO_CAMERAS_NAME})
target_link_libraries(${TWO_CAMERAS_NAME} PUBLIC cv_gl_lib)
set_target_properties(${TWO_CAMERAS_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


set(3D_RECON_NAME 3d_recon)
add_executable(${3D_RECON_NAME} 3d_recon.cpp ) #  sfm.cpp sfm_common.cpp
set_property(TARGET ${3D_RECON_NAME} PROPERTY CXX_STANDARD 11)
message("3d_recon_name = " ${3D_RECON_NAME})
target_link_libraries(${3D_RECON_NAME} PUBLIC cv_gl_lib cereal)
set_target_properties(${3D_RECON_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# Test Cereal
set(TS_NAME ts)
add_executable(${TS_NAME} apps/test_cereal.cpp test_class.cpp) #  sfm.cpp sfm_common.cpp
set_property(TARGET ${TS_NAME} PROPERTY CXX_STANDARD 11)
message("ts_name = " ${TS_NAME})
target_include_directories(${TS_NAME} PUBLIC ${PROJECT_INCLUDE_DIRS} )
target_link_libraries(${TS_NAME} PUBLIC cv_gl_lib cereal ) # cv_gl_lib
set_target_properties(${TS_NAME} PROPERTIES
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
