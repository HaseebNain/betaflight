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

#include <stdint.h>
#include <stdbool.h>

#include <platform.h>



#include "build/build_config.h"
#include "build/debug.h"

#include "blackbox/blackbox_io.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/compass/compass.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/max7456.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/vcd.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/motors.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/vtx_control.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"


// Default Settings for the RiotFC rev 1.802c
void targetConfiguration(void)
{
	// Motor Configurations
	// 3 = PWM_TYPE_MULTISHOT
	motorConfigMutable()->dev.motorPwmProtocol = 3;
	motorConfigMutable()->minthrottle = 1040;
	
	//Smartport Configurations
	//telemetryConfigMutable()->dev.telemetry_inversion = 0;
	//telemetryConfigMutable()->dev.sportHalfDuplex = 0;
	
	// Barometer Configuration
	//config->barometerConfig.baro_hardware = 3;

	
	// Motor Board Allignment Configurations
    boardAlignmentMutable()->rollDegrees = 0;
	boardAlignmentMutable()->pitchDegrees = 180;
	boardAlignmentMutable()->yawDegrees = 270;
	
	// Battery Configurations
	voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = 53;
	currentSensorADCConfigMutable()->scale = 250;
	
	// PID Configuration set to 8KHz
	pidConfigMutable()->pid_process_denom = 1;

	
	
	// PID Profile Configurations
	
	pidProfilesMutable(0)->pid[PID_ROLL].P = 43;
	pidProfilesMutable(0)->pid[PID_ROLL].I = 50;
	pidProfilesMutable(0)->pid[PID_ROLL].D = 20;
	pidProfilesMutable(0)->pid[PID_PITCH].P = 60;
	pidProfilesMutable(0)->pid[PID_PITCH].I = 55;
	pidProfilesMutable(0)->pid[PID_PITCH].D = 19;
	pidProfilesMutable(0)->pid[PID_YAW].P = 80;
	pidProfilesMutable(0)->pid[PID_YAW].I = 55;
	pidProfilesMutable(0)->pid[PID_YAW].D = 20;
	pidProfilesMutable(0)->pid[PID_ALT].P = 50;
	pidProfilesMutable(0)->pid[PID_ALT].I = 0;
	pidProfilesMutable(0)->pid[PID_ALT].D = 0;
	pidProfilesMutable(0)->pid[PID_POS].P = 15;
	pidProfilesMutable(0)->pid[PID_POS].I = 0;
	pidProfilesMutable(0)->pid[PID_POS].D = 0;
	pidProfilesMutable(0)->pid[PID_POSR].P = 34;
	pidProfilesMutable(0)->pid[PID_POSR].I = 14;
	pidProfilesMutable(0)->pid[PID_POSR].D = 53;
	pidProfilesMutable(0)->pid[PID_NAVR].P = 25;
	pidProfilesMutable(0)->pid[PID_NAVR].I = 33;
	pidProfilesMutable(0)->pid[PID_NAVR].D = 83;
	pidProfilesMutable(0)->pid[PID_LEVEL].P = 50;
	pidProfilesMutable(0)->pid[PID_LEVEL].I = 50;
	pidProfilesMutable(0)->pid[PID_LEVEL].D = 100;
	pidProfilesMutable(0)->pid[PID_MAG].P = 40;
	pidProfilesMutable(0)->pid[PID_VEL].P = 55;
	pidProfilesMutable(0)->pid[PID_VEL].I = 55;
	pidProfilesMutable(0)->pid[PID_VEL].D = 75;

	
	// Preset Port assignments
	serialConfigMutable()->portConfigs[0].identifier = serialPortIdentifiers[0];
	serialConfigMutable()->portConfigs[0].msp_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[0].gps_baudrateIndex = BAUD_57600;
	serialConfigMutable()->portConfigs[0].telemetry_baudrateIndex = BAUD_AUTO;
	serialConfigMutable()->portConfigs[0].blackbox_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_MSP;
	
	serialConfigMutable()->portConfigs[1].identifier = serialPortIdentifiers[1];
	serialConfigMutable()->portConfigs[1].msp_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[1].gps_baudrateIndex = BAUD_57600;
	serialConfigMutable()->portConfigs[1].telemetry_baudrateIndex = BAUD_AUTO;
	serialConfigMutable()->portConfigs[1].blackbox_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_MSP;
	
	serialConfigMutable()->portConfigs[2].identifier = serialPortIdentifiers[2];
	serialConfigMutable()->portConfigs[2].msp_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[2].gps_baudrateIndex = BAUD_57600;
	serialConfigMutable()->portConfigs[2].telemetry_baudrateIndex = BAUD_57600;
	serialConfigMutable()->portConfigs[2].blackbox_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_TELEMETRY_SMARTPORT;
	
	serialConfigMutable()->portConfigs[3].identifier = serialPortIdentifiers[3];
	serialConfigMutable()->portConfigs[3].msp_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[3].gps_baudrateIndex = BAUD_57600;
	serialConfigMutable()->portConfigs[3].telemetry_baudrateIndex = BAUD_AUTO;
	serialConfigMutable()->portConfigs[3].blackbox_baudrateIndex = BAUD_115200;
	serialConfigMutable()->portConfigs[3].functionMask = FUNCTION_RX_SERIAL;
	

}


