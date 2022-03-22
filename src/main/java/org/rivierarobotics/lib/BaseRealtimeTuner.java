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
import org.rivierarobotics.lib.shuffleboard.RSTab;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class BaseRealtimeTuner {
    protected final RSTab tab;
    protected final Map<String, Integer> listeners;

    public BaseRealtimeTuner(RSTab tab) {
        this.tab = tab;
        this.listeners = new HashMap<>();
    }

    public BaseRealtimeTuner initEntry(String title, double initValue,
                                       Consumer<Double> storageUpdater,
                                       Consumer<Double> actuatorUpdater) {
        tab.setEntry(title, initValue);
        Consumer<EntryNotification> func = callback -> {
            double value = callback.value.getDouble();
            storageUpdater.accept(value);
            actuatorUpdater.accept(value);

        };
        listeners.put(title, tab.getEntry(title).addListener(func, 0));
        return this;
    }

    public boolean hasInitialized() {
        return listeners.size() != 0;
    }

    public void destroy() {
        for (Map.Entry<String, Integer> listener : listeners.entrySet()) {
            tab.getEntry(listener.getKey()).removeListener(listener.getValue());
        }
        listeners.clear();
    }

    public interface ValueUpdater extends Consumer<Double> {
    }
}
