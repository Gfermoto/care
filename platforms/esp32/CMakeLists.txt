# C.A.R.E. ESP32 ESP-IDF Project
cmake_minimum_required(VERSION 3.16)

# Set the project name
set(PROJECT_NAME "care_esp32_radar")

# Include ESP-IDF CMake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(${PROJECT_NAME})