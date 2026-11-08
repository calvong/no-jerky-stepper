set(UNIT "${CMAKE_CURRENT_LIST_DIR}/../../unit")
idf_component_register(SRCS "test_main.cpp"
                            "${UNIT}/test_runner.cpp"
                            "${UNIT}/test_motion.cpp"
                            "${UNIT}/test_stepgen.cpp"
                            "${UNIT}/test_encoder.cpp"
                            "${UNIT}/test_retarget.cpp"
                            "${UNIT}/test_path.cpp"
                            "${UNIT}/test_group.cpp"
                            "${UNIT}/test_cruise.cpp"
                            "${UNIT}/test_feedback.cpp"
                       INCLUDE_DIRS "${UNIT}" "${CMAKE_CURRENT_LIST_DIR}/../../../src"
                       REQUIRES no-jerky-stepper unity)
