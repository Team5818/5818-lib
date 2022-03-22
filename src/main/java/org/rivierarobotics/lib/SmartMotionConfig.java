/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/**
 *
 * @see MotorUtil#setupSmartMotion(PIDConfig, int, SmartMotionConfig, CANSparkMax...)
 * @since 0.3.3
 */
public class SmartMotionConfig extends ControlModeConfig {
    private Double minVel = null;

    public SmartMotionConfig setMinVel(Double minVel) {
        this.minVel = minVel;
        return this;
    }

    public Double getMinVel() {
        return minVel;
    }

    public ControlModeConfig addStatusFrame(CANSparkMaxLowLevel.PeriodicFrame statusFrame) {
        return addStatusFrame(statusFrame.value);
    }
}
