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

package org.rivierarobotics.lib.shuffleboard;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * <p>Manages tiled data inside {@code Shuffleboard} and associated tabs.</p>
 *
 * <p>Uses separate instances of {@code RSTab} to store tabs, each
 * with a series of key/value entries of varying types. These can be updated
 * periodically (up to once every 100ms due to NetworkTables restrictions) through
 * the tab they belong to. Tabs do not enforce strict value types so long as they
 * are updated through the appropriate method(s).</p>
 *
 * @see RSTab
 * @see RSTable
 * @since 0.1.0
 */
public class RobotShuffleboard {
    private final Map<String, RSTab> tabs;

    /**
     * <p>Creates a new {@code Shuffleboard} manager.</p>
     *
     * <p>Implementations of this with Dagger and GradleRio-Redux should have a
     * singleton {@code RobotShuffleboard} added to the injection graph.
     * Multiple {@code RobotShuffleboard}s can be created and will function
     * correctly, but the idea is to use one object to control multiple tabs.</p>
     *
     * @since 0.1.0
     */
    public RobotShuffleboard() {
        this.tabs = new LinkedHashMap<>();
    }

    /**
     * <p>Gets a tab with a specified name.</p>
     *
     * <p>Will create a new tab via {@link #addTab(String...)} if the passed
     * name cannot be found in the internal tab entry map.</p>
     *
     * @param tabName the name of the tab to retrieve.
     * @return the tab with passed name.
     *
     * @since 0.1.0
     */
    public RSTab getTab(String tabName) {
        if (tabs.get(tabName) == null) {
            addTab(tabName);
        }
        return tabs.get(tabName);
    }

    /**
     * <p>Adds any number of tabs by name.</p>
     *
     * <p>Does nothing if the tab is already entered into the list.
     * It is suggested to use {@link #getTab(String)} for most operations.</p>
     *
     * @param tabNames the names of tabs to add.
     *
     * @since 0.1.0
     */
    public void addTab(String... tabNames) {
        for (String tab : tabNames) {
            if (!tabs.containsKey(tab)) {
                tabs.put(tab, new RSTab(tab));
            }
        }
    }

    /**
     * Removes a tab with a specified name.
     *
     * @param tabName the name of the tab to remove.
     * @return if the tab existed before removal.
     *
     * @since 0.1.0
     */
    public boolean removeTab(String tabName) {
        return tabs.remove(tabName) == null;
    }
}
