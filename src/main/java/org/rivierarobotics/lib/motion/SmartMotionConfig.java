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

package org.rivierarobotics.lib.motion;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.rivierarobotics.lib.MotorUtil;
import org.rivierarobotics.lib.pid.PIDConfig;

/**
 * <p>Stores a configuration of Smart Motion related constants
 * for use in {@code MotorUtil.setupSmartMotion(...)}.</p>
 *
 * <p>Includes the additional {@code minVel} field on top
 * of the base {@code MotionProfile} fields. This represents
 * the minimum smart motion velocity in ticks per 100 milliseconds.
 * Also has a method for adding status frames using Rev-specific
 * {@code CANSparkMaxLowLevel.PeriodicFrame} enum objects.</p>
 *
 * @see MotionProfile
 * @see MotorUtil#setupSmartMotion(PIDConfig, int, SmartMotionConfig, CANSparkMax...)
 * @since 0.4.0
 */
public class SmartMotionConfig extends MotionProfile {
    private Double minVel = null;

    public SmartMotionConfig setMinVel(Double minVel) {
        this.minVel = minVel;
        return this;
    }

    public Double getMinVel() {
        return minVel;
    }

    public MotionProfile addStatusFrame(CANSparkMaxLowLevel.PeriodicFrame statusFrame) {
        return addStatusFrame(statusFrame.value);
    }
}
