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

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE
	${app_sources}
)

include_directories("include")
# NORDIC SDK APP END


zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
