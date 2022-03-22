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

package org.rivierarobotics.lib.pid;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * Manages multiple different PID configurations on a
 * single CTRE motor controller. Does not require
 * manual setpoint and calculation control.
 * Previously {@code MultiPID}.
 *
 * @see PIDStore
 * @see WPIMultiPID
 * @since 0.1.0
 */
public class CTREMultiPID extends PIDStore {
    private final BaseTalon motor;

    /**
     * <p>Constructs a new PID manager with n set PID constant configurations
     * and a single CTRE motor controller.</p>
     *
     * <p>Automatically applies the configurations in passed order
     * and selects the 0-th configuration to be used.</p>
     *
     * @param motor the CTRE motor controller to manage.
     * @param configs the configurations to set on the controller.
     *
     * @since 0.1.0
     */
    public CTREMultiPID(BaseTalon motor, PIDConfig... configs) {
        super(configs);
        this.motor = motor;
        for (int slotIdx = 0; slotIdx < configs.length; slotIdx++) {
            applyTo(configs[slotIdx], motor, slotIdx);
        }
        selectConfig(0);
    }

    /**
     * Selects the configuration stored at a certain index
     * and switches the active motor PID profile.
     *
     * @param idx the index to select at.
     * @return if the selected index changed from the previous value.
     *
     * @since 0.1.0
     */
    @Override
    public boolean selectConfig(int idx) {
        boolean changedIdx = super.selectConfig(idx);
        if (changedIdx) {
            motor.selectProfileSlot(idx, 0);
        }
        return changedIdx;
    }

    /**
     * Applies a PIDF configuration to a CTRE motor controller.
     *
     * The slot number is [0, 3] as dictated by the 4 slots per controller.
     * Configurations only need to be applied once, then switched between with
     * {@code motor.selectProfileSlot(idx, 0)}. Note that the 0 represents
     * the primary controller. It is suggested to remain on the primary for quick
     * switching (i.e. position to velocity) and resort to auxiliary if more
     * than four configurations are needed (unlikely) or two controllers need
     * to be running simultaneously (not recommended).
     *
     * @param config the PIDF configuration to apply.
     * @param motor the CTRE motor to apply the PIDF configuration to.
     * @param slotIdx the index of the profile slot to apply the configuration to.
     *
     * @since 0.2.0
     */
    public static void applyTo(PIDConfig config, BaseTalon motor, int slotIdx) {
        motor.config_kP(slotIdx, config.getP());
        motor.config_kI(slotIdx, config.getI());
        motor.config_kD(slotIdx, config.getD());
        motor.config_kF(slotIdx, config.getF());
    }
}
