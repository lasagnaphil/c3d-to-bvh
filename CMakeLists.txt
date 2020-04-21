cmake_minimum_required(VERSION 3.14)
project(c3d-to-bvh)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(deps/c3dfile EXCLUDE_FROM_ALL)
add_subdirectory(deps/gengine EXCLUDE_FROM_ALL)

add_custom_target(lm_copy_resource_folder)
add_custom_command(TARGET lm_copy_resource_folder POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_BINARY_DIR}/resources)

add_subdirectory(src)
