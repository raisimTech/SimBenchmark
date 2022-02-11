message(STATUS "  raiSim for raiSim")

########################################################################################################################
# dependencies
find_package(raisim CONFIG REQUIRED)

########################################################################################################################
# sub-directories
if(BUILD_TEST)
    add_subdirectory(test)
endif(BUILD_TEST)
