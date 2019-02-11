cmake_minimum_required(VERSION 2.8)
project(EqualLoudness)
SET(CMAKE_BUILD_TYPE "Release")

include_directories(include)
add_executable(EqualLoudness main.cpp)

target_link_libraries(EqualLoudness -lm)