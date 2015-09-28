include_directories ( ${CLUtils_INCLUDE_DIR} 
                      ${GuidedFilter_INCLUDE_DIR}
                      ${RBC_INCLUDE_DIR}
                      ${EIGEN_INCLUDE_DIR}
                      ${ICP_INCLUDE_DIR} 
                      ${OCTOMAP_INCLUDE_DIR} )

add_library ( oclslamAlgorithms STATIC oclslam/algorithms.cpp )
add_library ( oclslamHelperFuncs STATIC oclslam/tests/helper_funcs.cpp )

add_dependencies ( oclslamAlgorithms  CLUtils GuidedFilter RBC Eigen ICP octomap )
add_dependencies ( oclslamHelperFuncs CLUtils GuidedFilter RBC Eigen ICP octomap )

# target_link_libraries ( oclslamAlgorithms ${RBC_LIBRARIES} )

target_include_directories ( 
    oclslamAlgorithms PUBLIC 
    ${COMMON_INCLUDES} 
)

target_include_directories ( 
    oclslamHelperFuncs PUBLIC 
    ${COMMON_INCLUDES} 
)

install ( DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION include )
install ( DIRECTORY ${PROJECT_BINARY_DIR}/lib/ DESTINATION lib/oclslam )
