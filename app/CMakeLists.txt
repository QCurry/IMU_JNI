cmake_minimum_required(VERSION 3.4.1)

SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -DNDEBUG")
SET(CMAKE_CXX_FLAGS_DEBUG  "-O3 -ffast-math")

find_library( log-lib log )

find_library( android-lib android )

# BMI160 DRIVER

include_directories(../app/libs/bmi160/include)

add_library(allanVarianceSampler SHARED
        src/main/cpp/main.cpp
        src/main/cpp/BMI160_IMU.cpp
        ../app/libs/bmi160/src/bmi160.c)

target_link_libraries(allanVarianceSampler ${log-lib} ${android-lib})