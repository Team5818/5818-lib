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
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * @see MotorUtil#setupMotionMagic(FeedbackDevice, PIDConfig, int, MotionMagicConfig, BaseTalon...)
 * @since 0.3.3
 */
public class MotionMagicConfig extends ControlModeConfig {
    private Integer sCurveStrength = null;

    public ControlModeConfig setSCurveStrength(Integer sCurveStrength) {
        this.sCurveStrength = sCurveStrength;
        return this;
    }

    public Integer getSCurveStrength() {
        return sCurveStrength;
    }

    public ControlModeConfig addStatusFrame(StatusFrameEnhanced statusFrame) {
        return addStatusFrame(statusFrame.value);
    }
}
