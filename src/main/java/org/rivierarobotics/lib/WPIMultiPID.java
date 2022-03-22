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

import edu.wpi.first.math.controller.PIDController;

import java.util.LinkedHashMap;
import java.util.function.Supplier;

/**
 * <p>Manages multiple different PID configurations on
 * synchronous WPILib PID controllers. Requires that setpoints
 * be passed and calculations occur within this object.</p>
 *
 * <p>Intended to be used for non-CTRE motor controllers
 * (e.g. SparkMAX) that do not support on-controller
 * motion profiling (e.g. MotionMagic).</p>
 *
 * @see PIDStore
 * @see CTREMultiPID
 * @since 0.2.0
 */
public class WPIMultiPID extends PIDStore {
    private final PIDController[] controllers;
    private final LinkedHashMap<Integer, Supplier<Double>> feedbackSuppliers;
    private boolean enabled;

    /**
     * <p>Constructs a new PID manager with n set PID constant configurations
     * and creates WPILib {@code PIDController} objects for each one.</p>
     *
     * <p>Does not use controllers unless called by {@code calculate()}.
     * Disabled by default, will only calculate with active controller.</p>
     *
     * @param configs the configurations to create controllers for.
     *
     * @since 0.2.0
     */
    public WPIMultiPID(PIDConfig... configs) {
        super(configs);
        this.controllers = new PIDController[configs.length];
        this.feedbackSuppliers = new LinkedHashMap<>();
        for (int slotIdx = 0; slotIdx < configs.length; slotIdx++) {
            PIDConfig config = configs[slotIdx];
            controllers[slotIdx] = new PIDController(config.getP(), config.getI(), config.getD());
            controllers[slotIdx].setTolerance(config.getTolerance());
        }
        disable();
    }

    /**
     * Enable the {@code MultiPID} instance.
     *
     * @see #isEnabled()
     * @since 0.3.2
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Disable the {@code MultiPID} instance. Prevents
     * calculations from running by toggling a flag.
     *
     * @see #isEnabled()
     * @since 0.2.0
     */
    public void disable() {
        enabled = false;
    }

    /**
     * <p>Determine if the {@code MultiPID} instance is enabled.</p>
     *
     * <p>The multicontroller can perform any action except calculate
     * a PID result while disabled.</p>
     *
     * @return if this multicontroller is enabled.
     *
     * @since 0.3.2
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * <p>Set the setpoint of the currently selected controller.</p>
     *
     * <p>This must be set before calling calculate, otherwise
     * no movement will occur. Ensure that the correct index is
     * selected before calling this method.</p>
     *
     * @param setpoint the new setpoint in units that are
     *                 the same as the indended feedback units.
     * @return if the setpoint was set to a valid controller.
     *
     * @since 0.2.0
     */
    public boolean setSetpoint(double setpoint) {
        boolean idxValid = super.isIndexValid();
        if (idxValid) {
            controllers[currentIdx].setSetpoint(setpoint);
        }
        return idxValid;
    }

    /**
     * <p>Provides a feedback supplier to the PID controller.</p>
     *
     * <p>Overload for {@link #supplyFeedback(int, Supplier)}.</p>
     *
     * @param mode the type identifier of the configuration to supply.
     * @param feedback the supplier to provide feedback
     *                 to the controller from.
     *
     * @see #supplyFeedback(int, Supplier)
     * @since 0.2.1
     */
    public void supplyFeedback(Mode mode, Supplier<Double> feedback) {
        supplyFeedback(mode.ordinal(), feedback);
    }

    /**
     * <p>Provides a feedback supplier to the PID controller.
     * Will automatically switch between suppliers when modes are changed.</p>
     *
     * @param idx the index to supply to.
     * @param feedback the supplier to provide feedback
     *                 to the controller from.
     *
     * @since 0.2.1
     */
    public void supplyFeedback(int idx, Supplier<Double> feedback) {
        feedbackSuppliers.put(idx, feedback);
    }

    /**
     * <p>Calculate the [-1, 1] motor power as given by the
     * currently selected PID controller. Automatically applies
     * feedback using provided suppliers.</p>
     *
     * <p>Will return a value of 0.0 if disabled.</p>
     *
     * @return the [-1, 1] power to set to the motor.
     *
     * @since 0.2.1
     */
    public double calculate() {
        if (!enabled) {
            return 0.0;
        }
        Supplier<Double> feedback = feedbackSuppliers.get(currentIdx);
        return feedback != null ? calculate(feedback.get()) : 0;
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
        return !super.isIndexValid() ? 0 : MathUtil.limit(
                controllers[currentIdx].calculate(feedback),
                configs[currentIdx].getRange()
        );
    }
}
