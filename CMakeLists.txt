# no-jerky-stepper — streaming minimum-jerk stepper motion control (ESP-IDF component)
#
# The motion core (src/motion, src/stepgen, src/core) is plain C++17 with no ESP-IDF
# dependency and is also built by the host test project in test/host.
# The ESP-IDF glue (RMT driver, public Stepper/Group API) follows in the next commit.

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


idf_component_register(SRCS ${NJ_CORE_SRCS}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
