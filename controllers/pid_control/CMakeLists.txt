get_filename_component(CONTROLLER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

set(WEBOTS_HOME "C:/Program Files/Webots")

link_directories(${WEBOTS_HOME}/lib/controller)

set(LIBRARIES
    m
    ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX}
)

include_directories(
    ${WEBOTS_HOME}/include/controller/c
    ${WEBOTS_HOME}/include/controller/cpp
)

add_executable(${CONTROLLER_NAME} ${SOURCES})
target_link_libraries(${CONTROLLER_NAME} ${LIBRARIES})

add_custom_command(TARGET ${CONTROLLER_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:${CONTROLLER_NAME}>
    ${CMAKE_CURRENT_SOURCE_DIR}/$<TARGET_FILE_NAME:${CONTROLLER_NAME}>
)