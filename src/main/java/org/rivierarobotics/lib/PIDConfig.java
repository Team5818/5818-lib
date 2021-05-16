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
 * Stores a PIDF configuration through loop gain constants.<br><br>
 *
 * Terms may be accessed or changed through standard getters/setters.
 * Contains helper method to apply terms to a given CTRE motor controller.
 * Should be used in all places where PIDF loops are needed.
 *
 * @since 0.1.0
 */
public class PIDConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double tolerance;
    private double pidRange;

    /**
     * Constructs a PID configuration with the specified loop gain constants.
     *
     * @param kP proportional gain; output proportional to current error.
     * @param kI integral gain; output based on accumulated error to exponentiate kP.
     * @param kD derivative gain; typically used for damping.
     * @param kF feed forward constant.
     * @param pidRange maximum (negated to get minimum) motor movement
     *                 percentage allowed for PIDF loop to set [-1, 1].
     *
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF, double pidRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.pidRange = pidRange;
    }

    /**
     * Constructs a PID configuration with the specified loop gain constants.
     * Has a default range value of [-1, 1] or full range.<br><br>
     *
     * Wrapper for {@link #PIDConfig(double, double, double, double, double)}.
     *
     * @see #PIDConfig(double, double, double, double, double)
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1.0);
    }

    /**
     * Constructs a PID configuration with the specified loop gain constants.
     * Has a default range value of [-1, 1] or full range and no feed forward.<br><br>
     *
     * Wrapper for {@link #PIDConfig(double, double, double, double, double)}.
     *
     * @see #PIDConfig(double, double, double, double, double)
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 1.0);
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getRange() {
        return pidRange;
    }

    public PIDConfig setP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDConfig setI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDConfig setD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDConfig setF(double kF) {
        this.kF = kF;
        return this;
    }

    public PIDConfig setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public PIDConfig setRange(double pidRange) {
        this.pidRange = pidRange;
        return this;
    }
}
