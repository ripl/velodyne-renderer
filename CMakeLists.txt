cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME velodyne-renderer)
include(cmake/pods.cmake)

# Necessary to build on OSX
find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB2 REQUIRED glib-2.0)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(BOT2_VIS REQUIRED bot2-vis)

if(NOT BOT2_VIS_FOUND)
    message("\n\n")
    message("bot2-vis not found.  Not building rrtstar renderer")
    message("\n")
    return()
endif()


add_subdirectory(src)
