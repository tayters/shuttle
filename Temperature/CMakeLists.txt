cmake_minimum_required(VERSION 3.10)


# set the project name
project(tempread)

# add the test library
add_subdirectory(Ezo_i2c_lib)

# add the executable
add_executable(tempread tempread.cpp)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

target_link_libraries(tempread PUBLIC Ezo_i2c_lib wiringPi)

target_include_directories(tempread PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/Ezo_i2c_lib"
                          )
