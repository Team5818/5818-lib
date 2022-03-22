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

public class MotorRealtimeTuner extends BaseRealtimeTuner {
    private final MotionProfile ctrlConfig;
    private final PIDConfig pidConfig;

    public MotorRealtimeTuner(RSTab tab, MotionProfile ctrlConfig, PIDConfig pidConfig) {
        super(tab);
        this.ctrlConfig = ctrlConfig;
        this.pidConfig = pidConfig;
    }

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

    public boolean initCTRE(BaseTalon motor) {
        return initCTRE(motor, 0);
    }

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

    public boolean initSpark(CANSparkMax motor) {
        return initSpark(motor, 0);
    }

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
}
