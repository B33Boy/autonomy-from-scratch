get_filename_component(CONTROLLER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

add_executable(${CONTROLLER_NAME} ${SOURCES})
target_link_libraries(${CONTROLLER_NAME} PRIVATE webots)