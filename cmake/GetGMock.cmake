# Enable ExternalProject CMake module
include(ExternalProject)
 
# Set default ExternalProject root directory
set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/ThirdParty)
 
# Add gmock
ExternalProject_Add(
    googlemock
    SVN_REPOSITORY http://googlemock.googlecode.com/svn/trunk/
    TIMEOUT 30
    
    CMAKE_ARGS -Dgtest_force_shared_crt=ON
    # Disable install step
    INSTALL_COMMAND ""
    # Wrap download, configure and build steps in a script to log output
    LOG_DOWNLOAD ON
    LOG_CONFIGURE ON
    LOG_BUILD ON)
 
# Specify include dir for googlemock and googletest
ExternalProject_Get_Property(googlemock source_dir)
ExternalProject_Get_Property(googlemock binary_dir)

set(GMOCK_INCLUDE_DIRS ${source_dir}/include ${source_dir}/gtest/include)
set(GMOCK_LIB_DIRS ${binary_dir} ${binary_dir}/gtest)
set(GMOCK_LIBRARIES gmock gmock_main gtest gtest_main)
set(GMOCK_FOUND TRUE)