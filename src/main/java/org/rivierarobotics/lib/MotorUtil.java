/*
 * This file is part of 5818-lib, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * Utility methods relating to robot motor movement.
 *
 * @since 0.1.0
 */
public class MotorUtil {
    private MotorUtil() {
    }

    /**
     * Configures Motion Magic motion profiling on given CTRE
     * Talon, Victor, or Falcon controlled motors.
     * Has a default PID slot index of 0 (primary).<br><br>
     *
     * Wrapper for {@link #setupMotionMagic(FeedbackDevice, PIDConfig, int, MotionMagicConfig, BaseTalon...)}.
     *
     * @see #setupMotionMagic(FeedbackDevice, PIDConfig, int, MotionMagicConfig, BaseTalon...)
     * @since 0.3.0
     */
    public static void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig,
                                        MotionMagicConfig mmConfig, BaseTalon... motors) {
        setupMotionMagic(sensor, pidConfig, 0, mmConfig, motors);
    }

    /**
     * Configures Motion Magic motion profiling on given CTRE
     * Talon, Victor, or Falcon controlled motors.<br><br>
     *
     * Uses the internal 1 kHz clock of the controller instead of the 20 ms
     * RoboRio clock. This is recommended as it removes the need to make
     * custom motion profiles, leading to faster turnaround times on subsystems.
     * As a warning, this first resets all motor settings to factory default
     * and then configures the feedback sensor based on the passed value.
     * As such is is recommended that this be the first motor configuration call
     * in any subsystem. Note that maximum velocity and acceleration will
     * not be set if <code>maxVel == 0</code>.
     *
     * @param sensor the sensor attached to the controller used for loop feedback.
     * @param pidConfig the PIDF and range values to use on the controller.
     * @param slotIdx the PID profile slot this configuration applies to.
     * @param mmConfig the specific non-PID configuration options for this controller.
     * @param motors the motors for which Motion Magic is enabled on.
     *
     * @see PIDConfig
     * @see MotionMagicConfig
     * @since 0.1.0
     */
    public static void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig, int slotIdx,
                                        MotionMagicConfig mmConfig, BaseTalon... motors) {
        for (BaseTalon motor : motors) {
            if (mmConfig.doReset()) {
                motor.configFactoryDefault();
            }
            motor.selectProfileSlot(slotIdx, 0);
            motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
                    mmConfig.getPeriod(), mmConfig.getTimeout());
            motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,
                    mmConfig.getPeriod(), mmConfig.getTimeout());
            if (sensor == FeedbackDevice.PulseWidthEncodedPosition
                    || sensor == FeedbackDevice.IntegratedSensor) {
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                        mmConfig.getPeriod(), mmConfig.getTimeout());
            }
            for (int statusFrame : mmConfig.getStatusFrames()) {
                motor.setStatusFramePeriod(statusFrame, mmConfig.getPeriod(), mmConfig.getTimeout());
            }
            motor.configSelectedFeedbackSensor(sensor, 0, mmConfig.getTimeout());

            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(pidConfig.getRange());
            motor.configPeakOutputReverse(-pidConfig.getRange());

            motor.config_kP(slotIdx, pidConfig.getP());
            motor.config_kI(slotIdx, pidConfig.getI());
            motor.config_kD(slotIdx, pidConfig.getD());
            motor.config_kF(slotIdx, pidConfig.getF());

            if (mmConfig.getIntegralZone() != null) {
                motor.config_IntegralZone(slotIdx, mmConfig.getIntegralZone(), mmConfig.getTimeout());
            }
            if (mmConfig.getSCurveStrength() != null) {
                motor.configMotionSCurveStrength(mmConfig.getSCurveStrength(), mmConfig.getTimeout());
            }

            if (mmConfig.getMaxVel() != null) {
                motor.configMotionCruiseVelocity(mmConfig.getMaxVel());
            }
            if (mmConfig.getMaxAccel() != null) {
                motor.configMotionAcceleration(mmConfig.getMaxAccel());
            }
        }
    }

    /**
     * Places limits on the range of CTRE Talon, Victor, or Falcon controlled motors.<br><br>
     *
     * Limits are defined in ticks and apply to both power and positional control sets.
     * It is still recommended that both are limited manually if possible. Note that
     * this is a hard stop (despite being a soft limit) and does not account for velocity
     * accumulated while moving. This value does not persist after power-off.
     *
     * @param forward the maximum ticks in the forward/positive direction.
     * @param reverse the minimum ticks in the reverse/backward/negative direction.
     * @param motors the motors to apply the soft limits onto.
     *
     * @since 0.1.0
     */
    public static void setSoftLimits(double forward, double reverse, BaseTalon... motors) {
        for (BaseTalon motor : motors) {
            motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
            motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
            motor.configForwardSoftLimitThreshold(forward);
            motor.configReverseSoftLimitThreshold(reverse);
            motor.configForwardSoftLimitEnable(true);
            motor.configReverseSoftLimitEnable(true);
        }
    }
}
