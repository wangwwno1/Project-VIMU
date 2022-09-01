px4_add_library(SoftwareSensor
        SoftwareSensor.h
        SoftwareSensor.cpp
        model_parameters.h
)

add_subdirectory(LinearStateModel)

target_compile_options(SoftwareSensor PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_link_libraries(SoftwareSensor PRIVATE px4_work_queue fault_detector LinearStateModel)
