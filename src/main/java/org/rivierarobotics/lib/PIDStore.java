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
 * Base class to manage multiple different PID
 * configurations.<br><br>
 *
 * Most often used for separate position and velocity PID
 * constants though it has the capability to be used for
 * any number of configurations. Uses the {@link PIDConfig}
 * system to store PID modes/constant sets.
 *
 * @see PIDConfig
 * @see Mode
 * @see CTREMultiPID
 * @see WPIMultiPID
 * @since 0.2.0
 */
public abstract class PIDStore {
    protected final PIDConfig[] configs;
    protected int currentIdx;

    /**
     * Constructs a new PID manager with n set PID constant configurations.
     * Automatically selects the first index.
     *
     * @param configs the configurations to set on the controller.
     *
     * @since 0.2.0
     */
    public PIDStore(PIDConfig[] configs) {
        this.configs = configs;
        selectConfig(0);
    }

    /**
     * Gets the configuration associated with a physics movement type.<br><br>
     *
     * Wrapper for {@link #getConfig(int)}.
     *
     * @param mode the type identifier of the configuration to get.
     * @return the stored PID configuration.
     *
     * @see #getConfig(int)
     * @see Mode
     * @since 0.2.0
     */
    public PIDConfig getConfig(Mode mode) {
        return getConfig(mode.ordinal());
    }

    /**
     * Gets the stored configuration at a certain index.
     *
     * @param idx the index to retrieve from.
     * @return the stored PID configuration.
     *
     * @since 0.2.0
     */
    public PIDConfig getConfig(int idx) {
        return configs[idx];
    }

    /**
     * Selects the configuration associated with a physics movement type.<br><br>
     *
     * Wrapper for {@link #selectConfig(int)}.
     *
     * @param mode the type identifier of the configuration to get.
     *
     * @see #selectConfig(int)
     * @see Mode
     * @since 0.2.0
     */
    public void selectConfig(Mode mode) {
        selectConfig(mode.ordinal());
    }

    /**
     * Selects the configuration stored at a certain index.
     *
     * @param idx the index to select at.
     * @return if the selected index changed from the previous value.
     *
     * @since 0.2.0
     */
    public boolean selectConfig(int idx) {
        boolean changedIdx = currentIdx != idx;
        if (changedIdx) {
            currentIdx = idx;
        }
        return changedIdx;
    }

    /**
     * Get the current movement mode/type configuration.<br><br>
     *
     * @return the current mode/type if the current
     *         index is valid, otherwise null.
     *
     * @since 0.2.1
     */
    public Mode getCurrentMode() {
        return isIdxValid() ? Mode.values()[currentIdx] : null;
    }

    /**
     * Gets the current index of the selected controller.
     *
     * @return the current selected index.
     *
     * @since 0.2.1
     */
    public int getCurrentIndex() {
        return currentIdx;
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
     * @since 0.2.1
     */
    public boolean isIdxValid() {
        return currentIdx > 0 && currentIdx < configs.length;
    }

    /**
     * Represents physics movement types (position, velocity, and acceleration)
     * for use in <code>MultiPID</code> managers.
     *
     * @since 0.2.0
     */
    public enum Mode {
        POSITION, VELOCITY, ACCELERATION
    }
}
