idf_component_register( SRCS "src/core/no_jerky_stepper.c" 
                             "src/platform/no_jerky_platform.c" 
                             "src/platform/esp32s3_rmt.c"
                             "src/motion/mjt.c"
                        
                        INCLUDE_DIRS "src/core" 
                                     "src/platform" 
                                     "src/motion"
                        
                        REQUIRES driver)
