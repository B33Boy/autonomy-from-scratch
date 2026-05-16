cmake_minimum_required(VERSION 3.31)
project(autonomy-from-scratch)

set(WEBOTS_HOME "C:/Program Files/Webots")

add_library(webots INTERFACE)

target_include_directories(webots INTERFACE 
    ${WEBOTS_HOME}/include/controller/c
    ${WEBOTS_HOME}/include/controller/cpp
)

target_link_directories(webots INTERFACE 
    ${WEBOTS_HOME}/lib/controller
)

target_link_libraries(webots INTERFACE
    m
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

add_subdirectory(controllers/line_follower)
add_subdirectory(controllers/pid_control)