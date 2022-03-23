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

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.rivierarobotics.lib.shuffleboard.RSTab;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

/**
 * <p>Allows tuning of values in realtime using {@code Shuffleboard}
 * entry listeners. Can be used for any value that needs to be
 * changed without re-deploying each time.</p>
 *
 * <p>To edit a value, change it in the {@code Shuffleboard} entry
 * it corresponds to after making a call to
 * {@link #initEntry(String, double, Consumer, Consumer)}.</p>
 *
 * <p>See {@link org.rivierarobotics.lib.motion.MotorRealtimeTuner}
 * for a usage of this for PID/Motion Profiling.</p>
 *
 * @see org.rivierarobotics.lib.motion.MotorRealtimeTuner
 * @since 0.4.0
 */
public class BaseRealtimeTuner {
    protected final RSTab tab;
    protected final Map<String, Integer> listeners;

    /**
     * Constructs a realtime tuner object and initializes the
     * listener ID map (used to remove listeners later).
     *
     * @param tab the tab to put all entries onto.
     *
     * @since 0.4.0
     */
    public BaseRealtimeTuner(RSTab tab) {
        this.tab = tab;
        this.listeners = new HashMap<>();
    }

    /**
     * <p>Initializes a single entry to be tuned.</p>
     *
     * <p>Call only once for each value, i.e. three times for a
     * basic PID loop. Intended to be used as a builder.</p>
     *
     * @param title the title of the shuffleboard entry.
     * @param initValue the initial value to set the shuffleboard entry to.
     * @param storageUpdater a storage updater function called when a callback
     *                       value is received. Does not perform any action
     *                       with the value. Will be skipped if null.
     * @param actuatorUpdater an actuator updater function called when a callback
     *                        value is received. Performs the action specified
     *                        with the value. Will be skipped if null.
     * @return the current {@code BaseRealtimeTuner} being modified.
     *
     * @since 0.4.0
     */
    public BaseRealtimeTuner initEntry(String title, double initValue,
                                       Consumer<Double> storageUpdater,
                                       Consumer<Double> actuatorUpdater) {
        tab.setEntry(title, initValue);
        Consumer<EntryNotification> func = callback -> {
            double value = callback.value.getDouble();
            if (storageUpdater != null) {
                storageUpdater.accept(value);
            }
            if (actuatorUpdater != null) {
                actuatorUpdater.accept(value);
            }
        };
        listeners.put(title, tab.getEntry(title).addListener(func, 0));
        return this;
    }

    /**
     * Determine whether there are values initialized to this tuner.
     *
     * @return true if this tuner is initialized
     *
     * @since 0.4.0
     */
    public boolean hasInitialized() {
        return listeners.size() != 0;
    }

    /**
     * Destroy the current tuner by removing all listeners.
     *
     * @param removeEntries true if entries should be removed from
     *                      NetworkTables. Entry representations on GUI
     *                      may persist even if removed from NetworkTables.
     *
     * @since 0.4.0
     */
    public void destroy(boolean removeEntries) {
        for (Map.Entry<String, Integer> listener : listeners.entrySet()) {
            NetworkTableEntry entry = tab.getEntry(listener.getKey());
            entry.removeListener(listener.getValue());
            if (removeEntries) {
                entry.delete();
            }
        }
        listeners.clear();
    }
}
