/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#define FC_VERSION_THRUST_DEV_LEVEL_MAJOR 1 //increment when a significant change is made
#define FC_VERSION_THRUST_DEV_LEVEL_MINOR 1 //increment when a bug is fixed

/* Version Log
Version 1.1
Enabled SDCard Ability
 -Enabled SDCard Usage
 -Introduced debugging information into SDCard.c
 -Removed support for BEEPER and MAG(AK8963)
 -Removed OPBL Target support
 -Cleaned Target.h file
 
 

Version 1.0
Updated Overall Functions and Defaults
 -Updated LED counts to account for removal of LEDs 7, 24, 35
 -Updated LED color to dictate major changes
 -Updated default mapping to SPEKTRUM Mapping (TAER1234)
 -Removed SD Card feature


Version 0.8
Updated Defaults
 -Updated initial port defines

Version 0.7
Barometer Inclusion
 -Updated defines to include barometer support.

Version 0.6
Betaflight Update
 -Code update (1/09/2016)
 -DSHOT1200 Introduction

Version 0.5
TESTING OSD Performance
DMA disabled
 

Version 0.4
Smartport Implementation
- Set defaults for smartport configuration
-- Telemetry inversion disabled
-- BIDIR disabled
-- Hardware: 39k drain pull down was changed to 10k

Version 0.3
Restructuring Codebase away from haseebnain

- Updated GYRO/ACCL allignments
-- Yaw ->270
-- Pitch -> 180
-- Removed board allignment from Target.h
- Include defines for custom LEDs
- Timer defines updated in Target.c
- Moved over presets from local version of Betaflight(V10)
- Motor preset for MULTISHOT
- Battery scaling adjusted
-- Voltage scale --> 53
-- Current scale --> 403
- PID has been set to 8kHz
- PID profile has been updated to include preset tune (Conrad Tune)

