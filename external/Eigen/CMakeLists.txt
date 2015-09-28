include ( ExternalProject )

set ( EIGEN_TARGET Eigen PARENT_SCOPE )

ExternalProject_Add ( 
    Eigen 
    HG_REPOSITORY https://bitbucket.org/eigen/eigen
    HG_TAG 3.2.4 
    PREFIX ${CMAKE_CURRENT_BINARY_DIR} 
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} 
    INSTALL_COMMAND "" 
    STEP_TARGETS COPY_HEADERS 
)

ExternalProject_Get_Property ( Eigen SOURCE_DIR )
ExternalProject_Get_Property ( Eigen BINARY_DIR )

ExternalProject_Add_Step (
    Eigen COPY_HEADERS 
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${SOURCE_DIR}/Eigen ${PROJECT_BINARY_DIR}/external/Eigen/include/eigen3 
    COMMENT "Copying Eigen headers to ${PROJECT_BINARY_DIR}/external/Eigen/include/eigen3"
    DEPENDEES download 
    ALWAYS 0
)

set ( EIGEN_INCLUDE_DIR ${PROJECT_BINARY_DIR}/external/Eigen/include/eigen3 PARENT_SCOPE )

set ( EIGEN_FOUND TRUE PARENT_SCOPE )

set ( EIGEN_ROOT ${SOURCE_DIR} PARENT_SCOPE )

install ( SCRIPT ${BINARY_DIR}/cmake_install.cmake )
