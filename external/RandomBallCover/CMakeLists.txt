include ( ExternalProject )

set ( RBC_TARGET RBC PARENT_SCOPE )

ExternalProject_Add ( 
    RBC DEPENDS ${CLUtils_TARGET} 
    GIT_REPOSITORY https://github.com/pAIgn10/RandomBallCover.git 
    PREFIX ${CMAKE_CURRENT_BINARY_DIR} 
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -DCLUtils_ROOT=${CLUtils_ROOT} # -DBUILD_TESTS=ON 
    INSTALL_COMMAND "" 
    STEP_TARGETS COPY_KERNELS 
)

ExternalProject_Get_Property ( RBC SOURCE_DIR )
ExternalProject_Get_Property ( RBC BINARY_DIR )

ExternalProject_Add_Step (
    RBC COPY_KERNELS 
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${SOURCE_DIR}/kernels/ ${PROJECT_BINARY_DIR}/kernels/RBC 
    COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_BINARY_DIR}/kernels/RBC/CMakeLists.txt 
    COMMENT "Copying RBC kernels to ${PROJECT_BINARY_DIR}/kernels/RBC"
    DEPENDEES download 
)

set ( RBC_INCLUDE_DIR ${SOURCE_DIR}/include PARENT_SCOPE )
set ( RBC_LIBS_DIR ${BINARY_DIR}/lib PARENT_SCOPE )

set ( 
    RBC_LIBRARIES 
    ${BINARY_DIR}/lib/libRBCAlgorithms.a 
    ${BINARY_DIR}/lib/libRBCHelperFuncs.a 
    PARENT_SCOPE 
)

set ( RBC_FOUND TRUE PARENT_SCOPE )

set ( RBC_ROOT ${SOURCE_DIR} PARENT_SCOPE )

install ( SCRIPT ${BINARY_DIR}/cmake_install.cmake )
