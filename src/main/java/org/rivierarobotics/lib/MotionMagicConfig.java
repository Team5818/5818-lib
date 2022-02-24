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

import java.util.ArrayList;
import java.util.List;

/**
 * <p>Stores a configuration of Motion Magic related constants
 * for use in {@code MotorUtil.setupMotionMagic()}.</p>
 *
 * <p>Values that are not specified by the constructor will not be
 * set, with the exception of timeout and period, which are
 * both 10ms by default.</p>
 *
 * <p>Methods are arranged as a builder. Daisy-chaining is encouraged.</p>
 *
 * @see MotorUtil#setupMotionMagic(FeedbackDevice, PIDConfig, int, MotionMagicConfig, BaseTalon...)
 * @since 0.3.0
 */
public class MotionMagicConfig {
    private final List<Integer> statusFrames;
    private boolean reset;
    private Integer maxVel;
    private Integer maxAccel;
    private Integer integralZone;
    private Integer sCurveStrength;
    private int timeoutMs;
    private int periodMs;

    /**
     * Constructs a Motion Magic configuration with the specified constants.
     *
     * @param statusFrames a list of integer status frames to set.
     * @param reset reset the controller to factory default if true.
     * @param maxVel maximum velocity of Motion Magic controller in ticks per 100ms.
     * @param maxAccel maximum acceleration of Motion Magic controller in ticks per 100ms.
     * @param integralZone constant for zone/range of integral term
     *                     in closed-loop error calculation.
     * @param sCurveStrength constant for S-Curve Strength (more in WPILib docs).
     * @param timeoutMs timeout of all controller calls in milliseconds.
     * @param periodMs period of all status frame set calls in milliseconds.
     *
     * @since 0.3.0
     */
    public MotionMagicConfig(List<Integer> statusFrames, boolean reset,
                             Integer maxVel, Integer maxAccel,
                             Integer integralZone, Integer sCurveStrength,
                             int timeoutMs, int periodMs) {
        this.statusFrames = statusFrames;
        this.reset = reset;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.integralZone = integralZone;
        this.sCurveStrength = sCurveStrength;
        this.timeoutMs = timeoutMs;
        this.periodMs = periodMs;
    }

    /**
     * <p>Constructs a Motion Magic configuration with the specified constants.
     * Initializes all values except timeout and period to null/false/empty.</p>
     *
     * <p>Overload for {@link #MotionMagicConfig(List, boolean, Integer, Integer, Integer, Integer, int, int)}.</p>
     *
     * @see #MotionMagicConfig(List, boolean, Integer, Integer, Integer, Integer, int, int)
     * @since 0.3.0
     */
    public MotionMagicConfig(int timeoutMs, int periodMs) {
        this(new ArrayList<>(), false, null, null, null, null, timeoutMs, periodMs);
    }

    /**
     * <p>Constructs a Motion Magic configuration with the specified constants.
     * Initializes timeout and period to 10ms each.</p>
     *
     * <p>Overload for {@link #MotionMagicConfig(int, int)}.</p>
     *
     * @see #MotionMagicConfig(int, int)
     * @since 0.3.0
     */
    public MotionMagicConfig() {
        this(10, 10);
    }

    public MotionMagicConfig addStatusFrame(StatusFrameEnhanced statusFrame) {
        return addStatusFrame(statusFrame.value);
    }

    public MotionMagicConfig addStatusFrame(int statusFrame) {
        statusFrames.add(statusFrame);
        return this;
    }

    public MotionMagicConfig setReset(boolean reset) {
        this.reset = reset;
        return this;
    }

    public MotionMagicConfig setMaxVel(Integer maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    public MotionMagicConfig setMaxAccel(Integer maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    public MotionMagicConfig setIntegralZone(Integer integralZone) {
        this.integralZone = integralZone;
        return this;
    }

    public MotionMagicConfig setSCurveStrength(Integer sCurveStrength) {
        this.sCurveStrength = sCurveStrength;
        return this;
    }

    public MotionMagicConfig setTimeout(int timeoutMs) {
        this.timeoutMs = timeoutMs;
        return this;
    }

    public MotionMagicConfig setPeriod(int periodMs) {
        this.periodMs = periodMs;
        return this;
    }

    public List<Integer> getStatusFrames() {
        return statusFrames;
    }

    public boolean doReset() {
        return reset;
    }

    public Integer getMaxVel() {
        return maxVel;
    }

    public Integer getMaxAccel() {
        return maxAccel;
    }

    public Integer getIntegralZone() {
        return integralZone;
    }

    public Integer getSCurveStrength() {
        return sCurveStrength;
    }

    public int getTimeout() {
        return timeoutMs;
    }

    public int getPeriod() {
        return periodMs;
    }
}
