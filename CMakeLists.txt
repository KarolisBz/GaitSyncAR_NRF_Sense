# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(GaitSyncAR_NRF_Sense)

target_sources(app PRIVATE 
    src/main.c 
    src/ble.c
    src/battery_monitor.c
    src/imu_step.c
    src/nfc.c
    src/device_role.c
)
target_include_directories(app PUBLIC include)