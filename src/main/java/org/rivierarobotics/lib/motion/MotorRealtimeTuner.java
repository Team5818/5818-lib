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

package org.rivierarobotics.lib.motion;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import org.rivierarobotics.lib.BaseRealtimeTuner;
import org.rivierarobotics.lib.pid.PIDConfig;
import org.rivierarobotics.lib.shuffleboard.RSTab;

import java.util.function.Consumer;

/**
 * <p>Realtime tuner for a motor (PID and Motion Profile).
 * Supports CTRE (Talon, Victor, Falcon) controllers
 * and Rev Robotics Spark MAX controllers.</p>
 *
 * <p>Allows for tuning constants without re-deploying,
 * especially useful for PIDF gains. This configuration
 * includes most basic tuning constants as well as
 * S-curve strength (CTRE) and minimum velocity (Spark)
 * but additional constants can be added using
 * {@link #initEntry(String, double, Consumer, Consumer)}.
 * For fewer, consider using the base class instead.</p>
 *
 * <p>Values are updated to an actator updater (the motor)
 * and a storage updater (PID config object) each time
 * the value changes and a callback occurs.</p>
 *
 * @see BaseRealtimeTuner
 * @see #initBaseController(ValueUpdater...)
 * @see #initCTRE(BaseTalon, int)
 * @see #initSpark(CANSparkMax, int)
 * @since 0.4.0
 */
public class MotorRealtimeTuner extends BaseRealtimeTuner {
    private final MotionProfile ctrlConfig;
    private final PIDConfig pidConfig;

    /**
     * Constructs a realtime motor tuner object.
     * Does not initialize any values.
     *
     * @param tab the tab to put all entries onto.
     * @param ctrlConfig the controller motion profile configuration.
     * @param pidConfig the controller PID(F) configuration.
     *
     * @see BaseRealtimeTuner#BaseRealtimeTuner(RSTab)
     * @since 0.4.0
     */
    public MotorRealtimeTuner(RSTab tab, MotionProfile ctrlConfig, PIDConfig pidConfig) {
        super(tab);
        this.ctrlConfig = ctrlConfig;
        this.pidConfig = pidConfig;
    }

    /**
     * <p>Initialize base values for a generic controller.</p>
     *
     * <p>Not intended to be called directly, but rather through
     * controller-specific methods in this class. Only call once,
     * otherwise values will be overwritten.</p>
     *
     * @param updaters all gain action updaters to use (motor config setters).
     *
     * @see ValueUpdater
     * @see #initCTRE(BaseTalon, int)
     * @see #initSpark(CANSparkMax, int)
     * @since 0.4.0
     */
    public void initBaseController(ValueUpdater... updaters) {
        initEntry("P Gain", pidConfig.getP(), pidConfig::setP, updaters[0]);
        initEntry("I Gain", pidConfig.getI(), pidConfig::setI, updaters[1]);
        initEntry("D Gain", pidConfig.getD(), pidConfig::setD, updaters[2]);
        initEntry("Feed Forward", pidConfig.getF(), pidConfig::setF, updaters[3]);
        initEntry("I Zone", ctrlConfig.getIntegralZone(), kIz -> ctrlConfig.setIntegralZone(kIz.intValue()), updaters[4]);
        initEntry("Max Output", pidConfig.getRange(), pidConfig::setRange, updaters[5]);
        initEntry("Min Output", -pidConfig.getRange(), minPwr -> pidConfig.setRange(-minPwr), updaters[6]);
        initEntry("Max Velocity", ctrlConfig.getMaxVel(), ctrlConfig::setMaxVel, updaters[7]);
        initEntry("Max Acceleration", ctrlConfig.getMaxAccel(), ctrlConfig::setMaxAccel, updaters[8]);
    }

    /**
     * <p>Initialize a CTRE motor controller (Talon, Victor, Falcon) for Motion Magic tuning.
     * Uses a default slot index of 0.</p>
     *
     * <p>Overload for {@link #initCTRE(BaseTalon, int)}.</p>
     *
     * @see #initCTRE(BaseTalon, int)
     * @since 0.4.0
     */
    public boolean initCTRE(BaseTalon motor) {
        return initCTRE(motor, 0);
    }

    /**
     * <p>Initialize a CTRE motor controller (Talon, Victor, Falcon) for Motion Magic tuning.</p>
     *
     * <p>This does not set up the motor for MotionMagic.</p>
     *
     * @param motor the motor to apply settings to.
     * @param slotIdx the slot on the selected motor to use.
     * @return if the configuration for this realtime tuner is a Motion Magic config.
     *
     * @since 0.4.0
     */
    public boolean initCTRE(BaseTalon motor, int slotIdx) {
        initBaseController(
                kP -> motor.config_kP(slotIdx, kP),
                kI -> motor.config_kI(slotIdx, kI),
                kD -> motor.config_kD(slotIdx, kD),
                kF -> motor.config_kF(slotIdx, kF),
                kIz -> motor.config_IntegralZone(slotIdx, kIz),
                maxPwr -> motor.configPeakOutputForward(maxPwr, ctrlConfig.getTimeout()),
                minPwr -> motor.configPeakOutputReverse(minPwr, ctrlConfig.getTimeout()),
                maxVel -> motor.configMotionCruiseVelocity(maxVel, ctrlConfig.getTimeout()),
                maxAccel -> motor.configMotionAcceleration(maxAccel, ctrlConfig.getTimeout())
        );
        boolean configValidType = ctrlConfig instanceof MotionMagicConfig;
        if (configValidType) {
            MotionMagicConfig mmConfig = (MotionMagicConfig) ctrlConfig;
            initEntry("S Curve Strength", mmConfig.getSCurveStrength(),
                    str -> mmConfig.setSCurveStrength(str.intValue()),
                    str -> motor.configMotionSCurveStrength(str.intValue(), mmConfig.getTimeout()));
        }
        return configValidType;
    }

    /**
     * <p>Initialize a Rev motor controller (Spark MAX) for Smart Motion tuning.
     * Uses a default slot index of 0.</p>
     *
     * <p>Overload for {@link #initSpark(CANSparkMax, int)}.</p>
     *
     * @see #initSpark(CANSparkMax, int)
     * @since 0.4.0
     */
    public boolean initSpark(CANSparkMax motor) {
        return initSpark(motor, 0);
    }

    /**
     * <p>Initialize a Rev motor controller (Spark MAX) for Smart Motion tuning.</p>
     *
     * <p>Settings applied in realtime are not set permanently (i.e. after power-off)
     * for Spark controllers. Use {@code motor.burnFlash()} when finished to do so.</p>
     *
     * @param motor the motor to apply settings to.
     * @param slotIdx the slot on the selected motor to use.
     * @return if the configuration for this realtime tuner is a Motion Magic config.
     *
     * @since 0.4.0
     */
    public boolean initSpark(CANSparkMax motor, int slotIdx) {
        SparkMaxPIDController pid = motor.getPIDController();
        initBaseController(
                kP -> pid.setP(kP, slotIdx),
                kI -> pid.setI(kI, slotIdx),
                kD -> pid.setD(kD, slotIdx),
                kF -> pid.setFF(kF, slotIdx),
                kIz -> pid.setIZone(kIz, slotIdx),
                maxPwr -> pid.setOutputRange(pid.getOutputMin(slotIdx), maxPwr),
                minPwr -> pid.setOutputRange(minPwr, pid.getOutputMax(slotIdx)),
                maxVel -> pid.setSmartMotionMaxVelocity(maxVel, slotIdx),
                maxAccel -> pid.setSmartMotionMaxAccel(maxAccel, slotIdx)
        );
        boolean configValidType = ctrlConfig instanceof SmartMotionConfig;
        if (configValidType) {
            SmartMotionConfig smConfig = (SmartMotionConfig) ctrlConfig;
            initEntry("Min Vel", smConfig.getMinVel(), smConfig::setMinVel,
                    minVel -> pid.setSmartMotionMinOutputVelocity(minVel.intValue(), slotIdx));
        }
        return configValidType;
    }

    /**
     * <p>Wrapper interface for a double consumer.
     * Used to avoid creating generic arrays in
     * {@link #initBaseController(ValueUpdater...)}.</p>
     *
     * <p>Not indented for end-user use. Use lambda expressions
     * or double consumer equivalents as in {@code init} methods.</p>
     *
     * @see #initBaseController(ValueUpdater...)
     * @see #initCTRE(BaseTalon, int)
     * @see #initSpark(CANSparkMax, int)
     * @since 0.4.0
     */
    public interface ValueUpdater extends Consumer<Double> {
    }
}
