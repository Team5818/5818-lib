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
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Helper command to set a position using a Motion Magic enabled CTRE motor controller.<br><br>
 *
 * Takes a subsystem per the WPILib standard <code>CommandBase</code> and
 * moves it (while being a <code>requirement</code>). Positions should be
 * supplied in ticks and timeout in seconds. Set timeout to any negative
 * number to make the command run until it is within tolerance only
 * (i.e. remove the timeout, not suggested). Similarly, setting front
 * and back limits to -1 respectively disable them. This is compatible
 * with {@link MotorUtil#setSoftLimits(double, double, BaseTalon...)}.
 * Behavior on limit overrun is to stop the command.<br><br>
 *
 * It is also suggested that this be a supertype for other wrapper commands.
 * That way subsystems can be injected by Dagger and still work off this.
 *
 * @param <T> the subsystem type; preserves methods for overriding in subclasses.
 *
 * @see MotorUtil#setupMotionMagic(FeedbackDevice, PIDConfig, int, MotionMagicConfig, BaseTalon...)
 * @since 0.1.0
 */
public abstract class MotionMagicSetPosition<T extends Subsystem> extends CommandBase {
    protected final T subsystem;
    protected final double forwardLimit;
    protected final double backwardLimit;
    protected final double maxError;
    protected final double setPoint;
    protected final double timeout;
    protected double startTime;

    /**
     * Constructs the command which goes to a position using Motion Magic.<br><br>
     *
     * All units should by default be in ticks. However, they may be any unit so long
     * as all passed values and method overrides use that same unit. This may include
     * angles (degrees or radians), positions (meters or inches), etc.
     *
     * @param subsystem the WPILib registered subsystem contains a CTRE motor configured to use MotionMagic.
     * @param fwdLimit maximum forward position allowed.
     * @param backLimit minimum backward position allowed.
     * @param setPoint target setpoint to move towards.
     * @param maxError maximum error allowed from the setpoint to consider "at target".
     * @param timeout maximum allowed time for command completion in seconds.
     *
     * @since 0.1.0
     */
    public MotionMagicSetPosition(T subsystem, double fwdLimit, double backLimit,
                                  double setPoint, double maxError, double timeout) {
        this.subsystem = subsystem;
        this.forwardLimit = fwdLimit;
        this.backwardLimit = backLimit;
        this.maxError = maxError;
        this.setPoint = setPoint;
        this.timeout = timeout;
        addRequirements(subsystem);
    }

    /**
     * Constructs the command which goes to a position using Motion Magic.
     * Disables forward and backward limits.<br><br>
     *
     * Wrapper for {@link #MotionMagicSetPosition(Subsystem, double, double, double, double, double)}.
     *
     * @see #MotionMagicSetPosition(Subsystem, double, double, double, double, double)
     * @since 0.1.0
     */
    public MotionMagicSetPosition(T subsystem, double setPoint, double maxError, double timeout) {
        this(subsystem, -1, -1, setPoint, maxError, timeout);
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
        setPosition(setPoint);
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Gets the position (default in ticks) of the subsystem.
     *
     * @return the current position.
     * @since 0.1.1
     */
    public abstract double getPosition();

    /**
     * Sets the position (default in ticks) to the subsystem.
     *
     * @param setPoint the setpoint to set.
     * @since 0.1.1
     */
    public abstract void setPosition(double setPoint);

    /**
     * Checks if the passed timeout duration has elapsed since start.
     *
     * @return if the timeout period has elapsed
     *
     * @since 0.1.1
     */
    public boolean hasTimedOut() {
        return Timer.getFPGATimestamp() - startTime > timeout;
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
        double pos = getPosition();
        return MathUtil.isWithinTolerance(pos, setPoint, maxError) || hasTimedOut()
                || (forwardLimit != -1 && pos >= forwardLimit)
                || (backwardLimit != -1 && pos <= backwardLimit);
    }
}
