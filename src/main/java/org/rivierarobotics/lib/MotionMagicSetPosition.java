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
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Helper command to set a position using a Motion Magic enabled CTRE motor controller.<br><br>
 *
 * Takes a subsystem per the WPILib standard <code>CommandBase</code> and
 * moves it (while being a <code>requirement</code>). Positions should be
 * supplied in ticks and timeout in seconds. Set timeout to any negative
 * number to make the command run until it is within tolerance only
 * (i.e. remove the timeout, not suggested). Similarly, setting front
 * and back limits to -1 respectively disable them. This is compatible
 * with {@link MotorUtil#setSoftLimits(int, int, BaseTalon...)}.
 * Behavior on limit overrun is to stop the command.<br><br>
 *
 * It is also suggested that this be a supertype for other wrapper commands.
 * That way subsystems can be injected by Dagger and still work off this.
 *
 * @param <T> the subsystem type; preserves methods for overriding in subclasses.
 *
 * @see MotorUtil#setupMotionMagic(FeedbackDevice, PIDConfig, int, BaseTalon...)
 * @since 0.1.0
 */
public class MotionMagicSetPosition<T extends SubsystemBase> extends CommandBase {
    protected final T subsystem;
    protected final DoubleSupplier getPosition;
    protected final DoubleConsumer setPosition;
    protected final double forwardLimit;
    protected final double backLimit;
    protected final double maxError;
    protected final double setPoint;
    protected final double timeout;
    private double start;

    /**
     * Constructs the command which goes to a position using Motion Magic.<br><br>
     *
     * All units should by default be in ticks. However, they may be any unit so long
     * as all suppliers, consumers, and passed values use that same unit. This may include
     * angles (degrees or radians), positions (meters or inches), etc.
     *
     * @param subsystem the WPILib registered subsystem contains a CTRE motor configured to use MotionMagic.
     * @param getPosition a supplier that retrieves the position to feed to the controller.
     * @param setPosition a consumer that sets the setpoint on the controller.
     * @param forwardLimit maximum forward position allowed.
     * @param backLimit minimum backward position allowed.
     * @param setPoint target setpoint to move towards.
     * @param maxError maximum error allowed from the setpoint to consider "at target".
     * @param timeout maximum allowed time for command completion.
     *
     * @since 0.1.0
     */
    public MotionMagicSetPosition(T subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition,
                                  double forwardLimit, double backLimit, double setPoint, double maxError, double timeout) {
        this.subsystem = subsystem;
        this.getPosition = getPosition;
        this.setPosition = setPosition;
        this.forwardLimit = forwardLimit;
        this.backLimit = backLimit;
        this.maxError = maxError;
        this.setPoint = setPoint;
        this.timeout = timeout;
        addRequirements(subsystem);
    }

    /**
     * Constructs the command which goes to a position using Motion Magic.
     * Disables forward and backward limits.<br><br>
     *
     * Wrapper for {@link #MotionMagicSetPosition(SubsystemBase, DoubleSupplier, DoubleConsumer, double, double, double, double, double)}.
     *
     * @see #MotionMagicSetPosition(SubsystemBase, DoubleSupplier, DoubleConsumer, double, double, double, double, double)
     * @since 0.1.0
     */
    public MotionMagicSetPosition(T subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition,
                                  double setPoint, double maxError, double timeout) {
        this(subsystem, getPosition, setPosition, -1, -1, setPoint, maxError, timeout);
    }

    /**
     * Initializes the command once. Standard WPILib command function override.<br><br>
     *
     * Passes the setpoint to the setpoint consumer on the subsystem.
     * (i.e. the motor takes the setpoint and starts going).
     * Timer to timeout starts after setpoint is set.
     *
     * @since 0.1.0
     */
    @Override
    public void initialize() {
        setPosition.accept(setPoint);
        start = Timer.getFPGATimestamp();
    }

    /**
     * Checks every loop if command has finished. Standard WPILib command function override.
     *
     * @return if the position is within the tolerance of the setpoint, the timeout period has
     * elapsed, the position is greater than the forward limit and it is enabled, or
     * the position is less than the backward limit and it is enabled.
     *
     * @since 0.1.0
     */
    @Override
    public boolean isFinished() {
        double pos = getPosition.getAsDouble();
        return MathUtil.isWithinTolerance(pos, setPoint, maxError) || (Timer.getFPGATimestamp() - start) > timeout
                || (forwardLimit != 1 && pos >= forwardLimit) || (backLimit != -1 && pos <= backLimit);
    }
}
