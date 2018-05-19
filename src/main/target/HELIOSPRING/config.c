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

#include <platform.h>

#include "config/config_eeprom.h"
#include "drivers/pwm_output.h"
#include "common/filter.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "fc/config.h"
#include "fc/fc_rc.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"

#define DEFAULT_PIDS_ROLL   {45, 50, 20}
#define DEFAULT_PIDS_PITCH  {45, 50, 22}
#define DEFAULT_PIDS_YAW    {45, 50, 8}

#define BUTTERED_PIDS_ROLL  (pid8_t){50, 50, 12}
#define BUTTERED_PIDS_PITCH (pid8_t){54, 50, 14}
#define BUTTERED_PIDS_YAW   (pid8_t){50, 50, 5}


void targetConfiguration(void) {
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    // armingConfigMutable()->gyro_cal_on_first_arm = true;
    rxConfigMutable()->rcInterpolation         = RC_SMOOTHING_MANUAL;
    rxConfigMutable()->rcInterpolationInterval = 14;
    rxConfigMutable()->rcInterpolationChannels = RC_INTERP_RPYT;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_MULTISHOT;
    gyroConfigMutable()->gyro_sync_denom  = 2; // 16KHZ GYRO
    pidConfigMutable()->pid_process_denom = 1; // 16KHZ PID
    systemConfigMutable()->cpu_overclock  = 1; //192MHz makes Multishot run a little better because of maths.
    
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->buttered_pids = pidProfileIndex > 0;
        if (pidProfile->buttered_pids) {
            pidProfile->pid[PID_ROLL]  = BUTTERED_PIDS_ROLL;
            pidProfile->pid[PID_PITCH] = BUTTERED_PIDS_PITCH;
            pidProfile->pid[PID_YAW]   = BUTTERED_PIDS_YAW;
        }
       
        /* Setpoints */
        // should't need to set these since they don't get init in gyro.c with USE_GYRO_IMUF
        // pidProfile->yaw_lpf_hz = 0;
        // pidProfile->dterm_lpf_hz = 0;    
        // pidProfile->dterm_notch_hz = 0;
        // pidProfile->dterm_notch_cutoff = 0;
        if (!pidProfileIndex) {
            pidProfile->dtermSetpointWeight   = 100;	
            pidProfile->setpointRelaxRatio    = 100;
        }
        pidProfile->dterm_filter_type     = FILTER_BIQUAD;
        pidProfile->dterm_filter_style    = KD_FILTER_NOSP;
        pidProfile->dterm_lpf_hz          = 65;
        pidProfile->dterm_notch_cutoff    = 0;
    }
}

