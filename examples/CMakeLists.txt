find_package ( GLUT REQUIRED )
find_package ( GLEW REQUIRED )
find_package ( libusb-1.0 REQUIRED )
find_package ( Freenect REQUIRED )
find_package ( Threads REQUIRED )

include_directories ( 
    ${OPENGL_INCLUDE_DIRS} 
    ${GLUT_INCLUDE_DIRS} 
    ${GLEW_INCLUDE_DIRS} 
    ${CLUtils_INCLUDE_DIR} 
    ${GuidedFilter_INCLUDE_DIR}
    ${RBC_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR} 
    ${ICP_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIR} 
    ${LIBUSB_1_INCLUDE_DIRS} 
    ${FREENECT_INCLUDE_DIR} 
)

add_executable ( 
    ${FNAME}_slam 
    slam.cpp 
    ${PROJECT_SOURCE_DIR}/src/glut_viewer.cpp 
    ${PROJECT_SOURCE_DIR}/src/ocl_processing.cpp 
    ${PROJECT_SOURCE_DIR}/src/freenect_rgbd.cpp 
)

add_executable ( ${FNAME}_octree_example octree_example.cpp )
add_executable ( ${FNAME}_coloroctree_example coloroctree_example.cpp )

add_dependencies ( ${FNAME}_slam CLUtils GuidedFiler RBC Eigen ICP octomap )
add_dependencies ( ${FNAME}_octree_example octomap )
add_dependencies ( ${FNAME}_coloroctree_example octomap )

target_link_libraries ( 
    ${FNAME}_slam 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${GLEW_LIBRARIES} 
    ${OPENCL_LIBRARIES} 
    ${CLUtils_LIBRARIES} 
    ${GuidedFilter_LIBRARIES} 
    ${ICP_LIBRARIES} 
    ${RBC_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
    ${FREENECT_LIBRARIES} 
    ${CMAKE_THREAD_LIBS_INIT} 
    oclslamAlgorithms 
)

target_link_libraries ( 
    ${FNAME}_octree_example 
    ${OPENCL_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
)

target_link_libraries ( 
    ${FNAME}_coloroctree_example 
    ${OPENCL_LIBRARIES} 
    ${OCTOMAP_LIBRARIES} 
)
