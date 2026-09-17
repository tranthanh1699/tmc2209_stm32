add_library(tmc2209 STATIC
    Src/tmc2209_motion.c
    Src/tmc2209.c
)

target_include_directories(tmc2209
    PUBLIC  Inc
)

# target_link_libraries(tmc2209 PUBLIC dev_common)
target_link_libraries(tmc2209 PRIVATE stm32cubemx)