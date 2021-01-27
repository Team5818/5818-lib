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

/**
 *
 * @since 0.1.0
 */
public class MathUtil {
    private static final double DEADBAND = 0.08;
    private static final double TICKS_PER_DEGREE = 4096.0 / 360;

    private MathUtil() {
    }

    /**
     * @param val
     * @return
     * @since 0.1.0
     */
    public static double fitDeadband(double val) {
        return fitDeadband(val, DEADBAND);
    }

    /**
     * @param val
     * @param deadband
     * @return
     * @since 0.1.0
     */
    public static double fitDeadband(double val, double deadband) {
        if (!(Math.abs(val) < deadband)) {
            if (val > 0) {
                if (val >= 1) {
                    return 1;
                } else {
                    return val - deadband;
                }
            } else if (val < 0) {
                if (val <= -1) {
                    return -1;
                } else {
                    return val + deadband;
                }
            }
        }
        return 0;
    }

    /**
     * @since 0.1.0
     */
    public static double wrapToCircle(double angle) {
        return wrapToCircle(angle, 360); //default 360 degrees/circle
    }

    /**
     * @param angle
     * @param fullCircle
     * @return
     * @since 0.1.0
     */
    public static double wrapToCircle(double angle, int fullCircle) {
        angle %= fullCircle;
        if (angle < 0) {
            return fullCircle + angle;
        } else {
            return angle;
        }
    }

    /**
     * @since 0.1.0
     */
    public static double limit(double value, double minmax) {
        return limit(value, -minmax, minmax);
    }

    /**
     * @param value
     * @param min
     * @param max
     * @return
     * @since 0.1.0
     */
    public static double limit(double value, double min, double max) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        } else {
            return value;
        }
    }

    /**
     * @since 0.1.0
     */
    public static double degreesToTicks(double degrees) {
        return degreesToTicks(degrees, TICKS_PER_DEGREE);
    }

    /**
     * @param degrees
     * @param ticksPerDegree
     * @return
     * @since 0.1.0
     */
    public static double degreesToTicks(double degrees, double ticksPerDegree) {
        return degrees * ticksPerDegree;
    }

    /**
     * @since 0.1.0
     */
    public static double ticksToDegrees(double ticks) {
        return ticksToDegrees(ticks, 1 / TICKS_PER_DEGREE);
    }

    /**
     * @param ticks
     * @param degreesPerTick
     * @return
     * @since 0.1.0
     */
    public static double ticksToDegrees(double ticks, double degreesPerTick) {
        return ticks * degreesPerTick;
    }

    /**
     * @param value
     * @param target
     * @param tolerance
     * @return
     * @since 0.1.0
     */
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    /**
     * @param values
     * @return
     * @since 0.1.0
     */
    public static double getMagnitude(double...values) {
        double sum = 0;
        for(double a: values) {
            sum += a * a;
        }
        return Math.sqrt(sum);
    }
}
