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
 * Manages multiple different PID configurations on a
 * single CTRE motor controller.<br><br>
 *
 * Most often used for separate position and velocity PID
 * constants though it has the capability to be used for
 * any number of configurations. Uses the {@link PIDConfig}
 * system to store PID modes/constant sets.
 *
 * @see PIDConfig
 * @see MultiPID.Type
 * @since 0.1.0
 */
public class MultiPID {
    private final BaseTalon motor;
    private final PIDConfig[] configs;
    private int currentIdx = 0;

    /**
     * Constructs a new PID manager with n set PID constant configurations
     * and a single CTRE motor controller.<br><br>
     *
     * Automatically applies the configurations in passed order
     * and selects the 0-th configuration to be used.
     *
     * @param motor the CTRE motor controller to manage.
     * @param configs the configurations to set on the controller.
     * @since 0.1.0
     */
    public MultiPID(BaseTalon motor, PIDConfig... configs) {
        this.motor = motor;
        this.configs = configs;
        for (int i = 0; i < configs.length; i++) {
            configs[i].applyTo(motor, i);
        }
        motor.selectProfileSlot(0, 0);
    }

    /**
     * Gets the configuration associated with a physics movement type.<br><br>
     *
     * Wrapper for {@link #getConfig(int)}.
     *
     * @param type the type identifier of the configuration to get.
     *
     * @see #getConfig(int)
     * @see MultiPID.Type
     * @since 0.1.0
     */
    public PIDConfig getConfig(MultiPID.Type type) {
        return getConfig(type.ordinal());
    }

    /**
     * Gets the stored configuration at a certain index.
     *
     * @param idx the index to retrieve from.
     * @return the stored PID configuration.
     *
     * @since 0.1.0
     */
    public PIDConfig getConfig(int idx) {
        return configs[idx];
    }

    /**
     * Selects the configuration associated with a physics movement type.<br><br>
     *
     * Wrapper for {@link #selectConfig(int)}.
     *
     * @param type the type identifier of the configuration to get.
     *
     * @see #selectConfig(int)
     * @see MultiPID.Type
     * @since 0.1.0
     */
    public void selectConfig(MultiPID.Type type) {
        selectConfig(type.ordinal());
    }

    /**
     * Selects the configuration stored at a certain index.
     *
     * @param idx the index to select at.
     *
     * @since 0.1.0
     */
    public void selectConfig(int idx) {
        if (currentIdx != idx) {
            motor.selectProfileSlot(idx, 0);
            currentIdx = idx;
        }
    }

    /**
     * Represents physics movement types (position, velocity, and acceleration)
     * for use in <code>MultiPID</code> managers.
     *
     * @since 0.1.0
     */
    public enum Type {
        POSITION, VELOCITY, ACCELERATION
    }
}
