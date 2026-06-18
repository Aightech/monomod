# MONOMOD LIS3DH node — XIAO ESP32-C3 + LIS3DH (accel + EMG on ADC1), WiFi.
# Reuses the shared components in ../components (lis3dh, monomod_protocol,
# monomod_core, monomod_drivers, monomod_net).
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lis3dh_node)
