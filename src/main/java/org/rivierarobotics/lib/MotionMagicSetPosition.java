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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

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

    public MotionMagicSetPosition(T subsystem, DoubleSupplier getPosition, DoubleConsumer setPosition,
                                  double setPoint, double maxError, double timeout) {
        this(subsystem, getPosition, setPosition, -1, -1, setPoint, maxError, timeout);
    }

    @Override
    public void initialize() {
        setPosition.accept(setPoint);
        start = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        double pos = getPosition.getAsDouble();
        return MathUtil.isWithinTolerance(pos, setPoint, maxError) || (Timer.getFPGATimestamp() - start) > timeout
                || (forwardLimit != 1 && backLimit != -1 && (pos >= forwardLimit || pos <= backLimit));
    }
}
