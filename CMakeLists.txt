# no-jerky-stepper — streaming minimum-jerk stepper motion control (ESP-IDF component)
#
# The motion core (src/motion, src/stepgen, src/core) is plain C++17 with no ESP-IDF
# dependency and is also built by the host test project in test/host.
# src/platform/esp32 + src/stepper.cpp + src/group.cpp are the ESP-IDF glue.

set(NJ_CORE_SRCS
    "src/motion/poly.cpp"
    "src/motion/min_jerk.cpp"
    "src/motion/strategy.cpp"
    "src/motion/trajectory.cpp"
    "src/motion/path_planner.cpp"
    "src/stepgen/step_generator.cpp"
    "src/stepgen/step_encoder.cpp"
    "src/core/axis.cpp"
    "src/core/group_planner.cpp")

set(NJ_ESP_SRCS
    "src/platform/esp32/rmt_step_channel.cpp"
    "src/platform/esp32/planner_task.cpp"
    "src/stepper.cpp"
    "src/group.cpp")

idf_component_register(SRCS ${NJ_CORE_SRCS} ${NJ_ESP_SRCS}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES esp_driver_gpio
                       PRIV_REQUIRES esp_driver_rmt)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
