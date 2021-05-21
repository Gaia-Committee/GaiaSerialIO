#==============================
# Requirements
#==============================

cmake_minimum_required(VERSION 3.10)

#==============================
# Project Settings
#==============================

if (NOT PROJECT_DECLARED)
    project("Gaia Serial IO" LANGUAGES CXX)
    set(PROJECT_DECLARED)
endif()

#==============================
# Unit Settings
#==============================

set(TARGET_NAME "GaiaSerialIO")

#==============================
# Command Lines
#==============================

set(CMAKE_CXX_STANDARD 17)

#==============================
# Source
#==============================

# Macro which is used to find .cpp files recursively.
macro(find_cpp path list_name)
    file(GLOB_RECURSE _tmp_list RELATIVE ${path} ${path}/*.cpp)
    set(${list_name})
    foreach(f ${_tmp_list})
        if(NOT f MATCHES "cmake-*")
            list(APPEND ${list_name} ${f})
        endif()
    endforeach()
endmacro()

# Macro which is used to find .hpp files recursively.
macro(find_hpp path list_name)
    file(GLOB_RECURSE _tmp_list RELATIVE ${path} ${path}/*.hpp)
    set(${list_name})
    foreach(f ${_tmp_list})
        if(NOT f MATCHES "cmake-*")
            list(APPEND ${list_name} ${f})
        endif()
    endforeach()
endmacro()

#------------------------------
# C++
#------------------------------

# C++ Source Files
find_cpp(${CMAKE_CURRENT_SOURCE_DIR} TARGET_SOURCE)
# C++ Header Files
find_hpp(${CMAKE_CURRENT_SOURCE_DIR} TARGET_HEADER)

#==============================
# Compile Targets
#==============================

# To lower the running cost, this module will be compiled as a static library.
add_library(${TARGET_NAME} SHARED ${TARGET_SOURCE} ${TARGET_HEADER})

# Enable 'DEBUG' Macro in Debug Mode
if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_compile_definitions(${TARGET_NAME} PRIVATE -DDEBUG)
endif()

#==============================
# External Modules
#==============================

# Exception module of Gaia.
target_link_libraries(${TARGET_NAME} PUBLIC "GaiaExceptions")
# Byte utility module of Gaia
target_link_libraries(${TARGET_NAME} PUBLIC "GaiaByteUtility")

#==============================
# Dependencies
#==============================

# Boost
find_package(Boost 1.65 REQUIRED COMPONENTS system)
target_include_directories(${TARGET_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} PUBLIC ${Boost_LIBRARIES})

# In Linux, 'Threads' need to explicitly linked.
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    find_package(Threads)
    target_link_libraries(${TARGET_NAME} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

#===============================
# Install Scripts
#===============================

# Install executable files and libraries to 'default_path/'.
install(TARGETS ${TARGET_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)
# Install header files to 'default_path/TARGET_NAME/'
install(DIRECTORY "." DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${TARGET_NAME}/ FILES_MATCHING PATTERN "*.hpp")