if ( BUILD_TESTS )

    find_package ( Threads REQUIRED )

    include_directories ( ${CLUtils_INCLUDE_DIR} 
                          ${GTEST_INCLUDE_DIRS}
                          ${RBC_INCLUDE_DIR}
                          ${OPENGL_INCLUDE_DIRS} )

    add_executable ( ${FNAME}_tests_oclslam testsOCLSLAM.cpp )

    add_dependencies ( ${FNAME}_tests_oclslam CLUtils googletest )

    target_link_libraries ( ${FNAME}_tests_oclslam LINK_PUBLIC ${CLUtils_LIBRARIES} 
                                                               oclslamHelperFuncs 
                                                               oclslamAlgorithms
                                                               ${OPENGL_LIBRARIES}
                                                               ${OPENCL_LIBRARIES}
                                                               ${GTEST_BOTH_LIBRARIES}
                                                               ${CMAKE_THREAD_LIBS_INIT} )

    add_test ( NAME ${FNAME}_tests_oclslam 
               COMMAND ${EXECUTABLE_OUTPUT_PATH}/${FNAME}_tests_oclslam 
               WORKING_DIRECTORY ${CMAKE_BINARY_DIR} )


    add_custom_target ( check COMMAND ${CMAKE_CTEST_COMMAND} --verbose )

endif ( BUILD_TESTS )
