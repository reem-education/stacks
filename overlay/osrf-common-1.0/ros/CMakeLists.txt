# fork off another cmake job to build and install osrf_msgs
# NOTE: for quick cycle hacking, just build it the "normal" way with "make"
ExternalProject_Add(osrf_msgs
                    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/osrf_msgs
                    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/osrf_msgs
                    CMAKE_ARGS
                      -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
                      -DOSRF_COMMON_VERSION:STRING=${OSRF_COMMON_VERSION}
		      ${CATKIN_BUILD_BINARY_PACKAGE_STRING}
                    INSTALL_COMMAND "")
install(CODE "execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/osrf_msgs install)")

