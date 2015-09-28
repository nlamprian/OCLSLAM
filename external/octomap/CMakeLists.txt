include ( ExternalProject )

set ( OCTOMAP_TARGET octomap PARENT_SCOPE )

ExternalProject_Add ( 
    octomap
    GIT_REPOSITORY https://github.com/OctoMap/octomap.git 
    # GIT_TAG devel-openmp
    PREFIX ${CMAKE_CURRENT_BINARY_DIR} 
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} # -DBUILD_TESTS=ON 
    INSTALL_COMMAND "" 
)

ExternalProject_Get_Property ( octomap SOURCE_DIR )
ExternalProject_Get_Property ( octomap BINARY_DIR )

set ( octomap_INCLUDE_DIR ${SOURCE_DIR}/octomap/include PARENT_SCOPE )
set ( octomap_LIBS_DIR ${SOURCE_DIR}/lib PARENT_SCOPE )

set ( 
    OCTOMAP_LIBRARIES 
    ${SOURCE_DIR}/lib/liboctomap.so 
    ${SOURCE_DIR}/lib/liboctomath.so 
    PARENT_SCOPE 
)

set ( OCTOMAP_FOUND TRUE PARENT_SCOPE )

set ( OCTOMAP_ROOT ${SOURCE_DIR} PARENT_SCOPE )

install ( SCRIPT ${BINARY_DIR}/cmake_install.cmake )
