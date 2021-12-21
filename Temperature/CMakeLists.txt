cmake_minimum_required(VERSION 3.10)

# set the project name
project(tempread)

add_subdirectory(Ezo_I2c_lib)


# add the executable
add_executable(tempread tempread.cpp)

target_link_Libraries(tempread PUBLIC Ezo_I2c_lib)

target_include_directories(tempread PUBLIC "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/Ezo_I2c_lib"
                          )  
