add_library(adc_tracker STATIC
    Src/adc_tracker.c
)

target_include_directories(adc_tracker
    PUBLIC  Inc
)

# target_link_libraries(adc_tracker PUBLIC dev_common)
target_link_libraries(adc_tracker PRIVATE stm32cubemx m)   # m: sqrtf/fabsf, must follow the archive on the link line