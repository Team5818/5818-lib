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
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.EntryNotification;
import org.rivierarobotics.lib.shuffleboard.RSTab;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class RealtimeTuner {
    private final RSTab tab;
    private final ControlModeConfig ctrlConfig;
    private final PIDConfig pidConfig;
    private final Map<String, Integer> allListeners;

    public RealtimeTuner(RSTab tab, ControlModeConfig ctrlConfig, PIDConfig pidConfig) {
        this.tab = tab;
        this.ctrlConfig = ctrlConfig;
        this.pidConfig = pidConfig;
        this.allListeners = new HashMap<>();
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
    
    public void initCTRE(BaseTalon motor) {
        initCTRE(motor, 0);
    }

    public void initCTRE(BaseTalon motor, int slotIdx) {
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
        if (ctrlConfig instanceof MotionMagicConfig) {
            MotionMagicConfig mmConfig = (MotionMagicConfig) ctrlConfig;
            initEntry("S Curve Strength", mmConfig.getSCurveStrength(),
                    str -> mmConfig.setSCurveStrength(str.intValue()),
                    str -> motor.configMotionSCurveStrength(str.intValue(), mmConfig.getTimeout()));
        }
    }
    
    public void initSpark(CANSparkMax motor) {
        initSpark(motor, 0);
    }
    
    public void initSpark(CANSparkMax motor, int slotIdx) {
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
        if (ctrlConfig instanceof SmartMotionConfig) {
            SmartMotionConfig smConfig = (SmartMotionConfig) ctrlConfig;
            initEntry("Min Vel", smConfig.getMinVel(), smConfig::setMinVel,
                    minVel -> pid.setSmartMotionMinOutputVelocity(minVel.intValue(), slotIdx));
        }
    }


    public RealtimeTuner initEntry(String title, double initValue,
                                   Consumer<Double> storageUpdater,
                                   Consumer<Double> actuatorUpdater) {
        tab.setEntry(title, initValue);
        Consumer<EntryNotification> func = callback -> {
            double value = callback.value.getDouble();
            storageUpdater.accept(value);
            actuatorUpdater.accept(value);

        };
        allListeners.put(title, tab.getEntry(title).addListener(func, 0));
        return this;
    }

    public boolean hasInitialized() {
        return allListeners.size() != 0;
    }

    public void destroy() {
        for (Map.Entry<String, Integer> listener : allListeners.entrySet()) {
            tab.getEntry(listener.getKey()).removeListener(listener.getValue());
        }
        allListeners.clear();
    }

    public interface ValueUpdater extends Consumer<Double> {
    }
}
