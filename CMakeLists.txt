# Create a shared library libhello.so with a single source file
add_definitions(-std=gnu99)

add_library(velodyne-renderer SHARED renderer_velodyne.c)

set(REQUIRED_LIBS velodyne bot2-core bot2-vis bot2-frames bot2-param-client path-utils)
pods_use_pkg_config_packages(velodyne-renderer ${REQUIRED_LIBS})

target_link_libraries(velodyne-renderer
    ${OPENGL_LIBRARIES}
    ${LCMTYPES_LIBS})

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(velodyne-renderer PROPERTIES SOVERSION 1)

# make the header public
# install it to include/velodyne
pods_install_headers(renderer_velodyne.h DESTINATION velodyne)

# make the library public
pods_install_libraries(velodyne-renderer)

# create a pkg-config file for the library, to make it for other software to
# use it.
# Velodyne Driver

pods_install_pkg_config_file(velodyne-renderer
    CFLAGS
    LIBS -lvelodyne-renderer
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)

add_executable(velodyne-viewer main.c)

pods_use_pkg_config_packages (velodyne-viewer
    bot2-core
    bot2-param-client
    velodyne-renderer
    glib-2.0
    bot2-vis
    bot2-frames
    lcm
    )

pods_install_executables(velodyne-viewer)
