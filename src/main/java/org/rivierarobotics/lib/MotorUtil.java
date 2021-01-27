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
 *
 * @since 0.1.0
 */
public class MotorUtil {
    private MotorUtil() {
    }

    /**
     * @param sensor
     * @param pidConfig
     * @param maxVel
     * @param motors
     * @since 0.1.0
     */
    public static void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig, int maxVel, BaseTalon... motors) {
        int periodMs = 10;
        int timeoutMs = 10;
        for (BaseTalon motor : motors) {
            motor.configFactoryDefault();
            motor.selectProfileSlot(0, 0);
            motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, periodMs, timeoutMs);
            motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, periodMs, timeoutMs);
            if (sensor == FeedbackDevice.PulseWidthEncodedPosition || sensor == FeedbackDevice.IntegratedSensor) {
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, periodMs, timeoutMs);
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, periodMs, timeoutMs);
            }
            motor.configSelectedFeedbackSensor(sensor, 0, timeoutMs);

            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(pidConfig.getRange());
            motor.configPeakOutputReverse(-pidConfig.getRange());

            motor.config_kP(0, pidConfig.getP());
            motor.config_kI(0, pidConfig.getI());
            motor.config_kD(0, pidConfig.getD());
            motor.config_kF(0, pidConfig.getF());

            if (maxVel != 0) {
                motor.configMotionCruiseVelocity(maxVel);
                motor.configMotionAcceleration(maxVel);
            }
        }
    }

    /**
     * @param forward
     * @param reverse
     * @param motors
     * @since 0.1.0
     */
    public static void setSoftLimits(int forward, int reverse, BaseTalon... motors) {
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
