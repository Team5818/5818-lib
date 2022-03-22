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

import java.util.ArrayList;
import java.util.List;

/**
 * <p>Stores a configuration of motion profiling related constants
 * for use in {@code MotorUtil.setup[...]()}.</p>
 *
 * <p>Values that are not specified by the constructor will not be
 * set, with the exception of timeout and period, which are
 * both 10ms by default.</p>
 *
 * <p>Methods are arranged as a builder. Daisy-chaining is encouraged.</p>
 *
 * @see MotionMagicConfig
 * @see SmartMotionConfig
 * @since 0.3.0
 */
public class MotionProfile {
    private final List<Integer> statusFrames;
    private boolean reset;
    private Double maxVel;
    private Double maxAccel;
    private Integer integralZone;
    private int timeoutMs;
    private int periodMs;

    /**
     * Constructs a motion profiling configuration with the specified constants.
     *
     * @param statusFrames a list of integer status frames to set.
     * @param reset reset the controller to factory default if true.
     * @param maxVel maximum velocity of motion profile in ticks per 100ms.
     * @param maxAccel maximum acceleration of motion profile in ticks per 100ms.
     * @param integralZone constant for zone/range of integral term
     *                     in closed-loop error calculation.
     * @param timeoutMs timeout of all controller calls in milliseconds.
     * @param periodMs period of all status frame set calls in milliseconds.
     *
     * @since 0.3.0
     */
    public MotionProfile(List<Integer> statusFrames, boolean reset,
                         Double maxVel, Double maxAccel,
                         Integer integralZone,
                         int timeoutMs, int periodMs) {
        this.statusFrames = statusFrames;
        this.reset = reset;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.integralZone = integralZone;
        this.timeoutMs = timeoutMs;
        this.periodMs = periodMs;
    }

    /**
     * <p>Constructs a motion profiling configuration with the specified constants.
     * Initializes all values except timeout and period to null/false/empty.</p>
     *
     * <p>Overload for {@link #MotionProfile(List, boolean, Double, Double, Integer, int, int)}.</p>
     *
     * @see #MotionProfile(List, boolean, Double, Double, Integer, int, int)
     * @since 0.3.0
     */
    public MotionProfile(int timeoutMs, int periodMs) {
        this(new ArrayList<>(), false, null, null, null, timeoutMs, periodMs);
    }

    /**
     * <p>Constructs a motion profiling configuration with the specified constants.
     * Initializes timeout and period to 10ms each.</p>
     *
     * <p>Overload for {@link #MotionProfile(int, int)}.</p>
     *
     * @see #MotionProfile(int, int)
     * @since 0.3.0
     */
    public MotionProfile() {
        this(10, 10);
    }

    public MotionProfile addStatusFrame(int statusFrame) {
        statusFrames.add(statusFrame);
        return this;
    }

    public MotionProfile addStatusFrames(int... frames) {
        for (int frame : frames) {
            statusFrames.add(frame);
        }
        return this;
    }

    public MotionProfile setReset(boolean reset) {
        this.reset = reset;
        return this;
    }

    public MotionProfile setMaxVel(Double maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    public MotionProfile setMaxAccel(Double maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    public MotionProfile setIntegralZone(Integer integralZone) {
        this.integralZone = integralZone;
        return this;
    }

    public MotionProfile setTimeout(int timeoutMs) {
        this.timeoutMs = timeoutMs;
        return this;
    }

    public MotionProfile setPeriod(int periodMs) {
        this.periodMs = periodMs;
        return this;
    }

    public List<Integer> getStatusFrames() {
        return statusFrames;
    }

    public boolean doReset() {
        return reset;
    }

    public Double getMaxVel() {
        return maxVel;
    }

    public Double getMaxAccel() {
        return maxAccel;
    }

    public Integer getIntegralZone() {
        return integralZone;
    }

    public int getTimeout() {
        return timeoutMs;
    }

    public int getPeriod() {
        return periodMs;
    }
}
