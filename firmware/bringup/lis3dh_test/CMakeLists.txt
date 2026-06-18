# Standalone bring-up test: probe a LIS3DH on SPI, signal result on a NeoPixel.
# Reuses the proven `lis3dh` driver from the main firmware's components dir.
# Only the specific components are listed (not the whole components/ dir) so the
# networking components — which need the managed mdns dep — aren't pulled in.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/lis3dh"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/monomod_board")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lis3dh_test)
