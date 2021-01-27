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

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 *
 * @since 0.1.0
 */
public class PIDConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double pidRange;

    /**
     * @param kP
     * @param kI
     * @param kD
     * @param kF
     * @param pidRange
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
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1.0);
    }

    /**
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

    public PIDConfig setRange(double pidRange) {
        this.pidRange = pidRange;
        return this;
    }

    /**
     * @param motor
     * @param slotIdx
     * @since 0.1.0
     */
    public void applyTo(BaseTalon motor, int slotIdx) {
        motor.config_kP(slotIdx, kP);
        motor.config_kI(slotIdx, kI);
        motor.config_kD(slotIdx, kD);
        motor.config_kF(slotIdx, kF);
    }
}
