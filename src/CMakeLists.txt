file(GLOB LM_C3D_SOURCES c3d/*.cpp)
file(GLOB LM_DATASET_SOURCES dataset/*.cpp)
file(GLOB LM_NN_SOURCES nn/*.cpp)

function(PREPARE_TARGET TARGET)
    target_compile_definitions(${TARGET} PRIVATE -DCUSTOM_SYS_PATH=\"${CMAKE_HOME_DIRECTORY}/venv/lib/python3.7/site-packages\")
    target_link_libraries(${TARGET} gengine stdc++fs pybind11::embed
            xtensor xtensor::optimize xtensor::use_xsimd xtensor-python xtensor-io)
    add_dependencies(${TARGET} lm_copy_resource_folder)
endfunction()

add_executable(bvh_viewer bvh_viewer.cpp ${LM_C3D_SOURCES})
target_link_libraries(bvh_viewer gengine c3dfile stdc++fs)
add_dependencies(bvh_viewer lm_copy_resource_folder)

add_executable(bvh_converter bvh_converter.cpp ${LM_C3D_SOURCES})
target_link_libraries(bvh_converter gengine c3dfile stdc++fs)
add_dependencies(bvh_converter lm_copy_resource_folder)

add_executable(bvh_converter_test bvh_converter_test.cpp ${LM_C3D_SOURCES})
target_link_libraries(bvh_converter_test gengine c3dfile stdc++fs)
add_dependencies(bvh_converter_test lm_copy_resource_folder)