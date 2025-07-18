# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html

idf_component_register(SRCS "motors.c" "motors_def_cf2.c"
                    INCLUDE_DIRS "include"
                    REQUIRES crazyflie)
add_compile_definitions(MOTORS_TEST_GPIO)
