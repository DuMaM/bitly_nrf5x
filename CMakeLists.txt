#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

IF(DEFINED CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP)
set(DTC_OVERLAY_FILE "nrf5340dk_nrf5340_cpuapp.overlay")
ENDIF()

IF(DEFINED CONFIG_BOARD_PARTICLE_XENON)
set(DTC_OVERLAY_FILE "particle_xenon.overlay")
ENDIF()

# load zephyr features and configure project
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# SERVICE APP START
FILE(GLOB app_service_sources src/app_services/*.c)
zephyr_sources(${app_service_sources})
zephyr_include_directories(include/app_services)
# SERVICE APP END

# NORDIC SDK APP START
FILE(GLOB app_sources src/app/*.c)
target_sources(app PRIVATE ${app_sources})
include_directories("include/app")
# NORDIC SDK APP END

# https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.0.2/nrf/ug_multi_image.html?highlight=child_image
zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
# add_child_image(
#    NAME hci_rpmsg
#    SOURCE_DIR ${ZEPHYR_BASE}/samples/bluetooth/hci_rpmsg
#    DOMAIN CPUNET
#    )