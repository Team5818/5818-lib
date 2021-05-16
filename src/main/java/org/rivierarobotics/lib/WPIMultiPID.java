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

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Manages multiple different PID configurations on
 * synchronous WPILib PID controllers. Requires that setpoints
 * be passed and calculations occur within this object.<br><br>
 *
 * Intended to be used for non-CTRE motor controllers
 * (e.g. SparkMAX) that do not support on-controller
 * motion profiling (e.g. MotionMagic).
 *
 * @see PIDStore
 * @see CTREMultiPID
 * @since 0.2.0
 */
public class WPIMultiPID extends PIDStore {
    private final PIDController[] controllers;

    /**
     * Constructs a new PID manager with n set PID constant configurations
     * and creates WPILib <code>PIDController</code> objects for each one.<br><br>
     *
     * Does not use controllers unless called by <code>calculate()</code>.
     * Disabled by default, will only calculate with active controller.
     *
     * @param configs the configurations to create controllers for.
     *
     * @since 0.2.0
     */
    public WPIMultiPID(PIDConfig... configs) {
        super(configs);
        this.controllers = new PIDController[configs.length];
        for (int slotIdx = 0; slotIdx < configs.length; slotIdx++) {
            PIDConfig config = configs[slotIdx];
            controllers[slotIdx] = new PIDController(config.getP(), config.getI(), config.getD());
            controllers[slotIdx].setTolerance(config.getTolerance());
        }
        disable();
    }

    /**
     * Disable the <code>MultiPID</code> instance. Prevents
     * calculations from running by selecting an invalid index.
     *
     * @since 0.2.0
     */
    public void disable() {
        currentIdx = -1;
    }

    /**
     * Set the setpoint of the currently selected controller.<br><br>
     *
     * This must be set before calling calculate, otherwise
     * no movement will occur. Ensure that the correct index is
     * selected before calling this method.
     *
     * @param setpoint the new setpoint in units that are
     *                 the same as the indended feedback units.
     * @return if the setpoint was set to a valid controller.
     *
     * @since 0.2.0
     */
    public boolean setSetpoint(double setpoint) {
        boolean idxValid = isIdxValid();
        if (idxValid) {
            controllers[currentIdx].setSetpoint(setpoint);
        }
        return idxValid;
    }

    /**
     * Calculate the [-1, 1] motor power as given by the
     * currently selected PID controller. Should be called
     * periodically and set when in use.
     *
     * @param feedback the current feedback from the motor,
     *                in units matching the setpoint
     *                (e.g. ticks/100ms for velocity)
     * @return the [-1, 1] power to set to the motor.
     *
     * @since 0.2.0
     */
    public double calculate(double feedback) {
        return !isIdxValid() ? 0 : MathUtil.limit(
                controllers[currentIdx].calculate(feedback),
                configs[currentIdx].getRange()
        );
    }

    /**
     * Determine if the currently selected index is a valid
     * index in the configuration array.<br><br>
     *
     * Invalid indices may occur when disabled <code>-1 < 0</code>
     * or if passed to <code>selectConfig(int idx)</code>.
     *
     * @return if the currently selected index is valid.
     *
     * @since 0.2.0
     */
    private boolean isIdxValid() {
        return currentIdx > 0 && currentIdx < configs.length;
    }
}