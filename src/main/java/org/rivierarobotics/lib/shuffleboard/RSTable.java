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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 *
 * Represents a <code>Shuffleboard</code>-based table displayed on a <code>RSTab</code>.<br><br>
 *
 * Uses <code>BuiltInLayouts.kList</code> to create a new list (table) layout on a tab.
 * Entries are then created using <code>withWidget("NetworkTableTree")</code>
 * or retrieved from the internal entries list.
 * Positioning and sizing of the table is managed by <code>RSTileOptions</code>.<br><br>
 *
 * Methods are arranged as a builder. Daisy-chaining is encouraged.
 *
 * @see RSTab
 * @see RSTileOptions
 * @since 0.1.0
 */
public class RSTable {
    private final LinkedHashMap<String, NetworkTableEntry> entries;
    private final String tableName;
    private final ShuffleboardLayout layout;
    private final RSTileOptions options;

    /**
     * Constructs a new table on a given <code>RSTab</code>.<br><br>
     *
     * Uses the <code>BuiltInLayouts.kList</code> enum to signal a table.
     * (The name indicates a list but is actually a key/value list).
     * The table label is hidden by default (clutters UI).
     * Does not automatically add the tab's contents to the table, for
     * that see {@link #addTabData(RSTab)}.
     *
     * @param tableName the name/title of the table to be created.
     * @param tab the tab to place the new table onto.
     * @param options placement and sizing options for the <code>RSTable</code>.
     *
     * @since 0.1.0
     */
    public RSTable(String tableName, RSTab tab, RSTileOptions options) {
        this.tableName = tableName;
        this.options = options;
        this.layout = tab.getAPITab()
                .getLayout(tableName, BuiltInLayouts.kList)
                .withSize(options.getWidth(), options.getHeight())
                .withPosition(options.getPosX(), options.getPosY())
                .withProperties(Map.of("Label position", "HIDDEN"));
        this.entries = new LinkedHashMap<>();
    }

    /**
     * Adds a single key/title and value to the current table.<br><br>
     *
     * First checks for a matching key is in the entry log for this table.
     * If present, the entry's value will be updated to reflect the value passed.
     * If not present, a new entry will be created by adding it to the table
     * layout created in the constructor.
     * Note that <code>withWidget("NetworkTableTree")</code> is called to ensure
     * the key/value appears within the table and not as a separate tile.
     *
     * @param name the name of the key/value pair to add.
     * @param value the value of the key/value pair to add.
     *
     * @since 0.1.0
     */
    public RSTable setEntry(String name, Object value) {
        if (entries.containsKey(name)) {
            entries.get(name).forceSetValue(value);
        } else {
            entries.put(name, layout.add(tableName + "/" + name, value)
                    .withWidget("Network Table Tree")
                    .getEntry());
        }
        return this;
    }

    /**
     * Adds all key/value tiles from a passed tab to the current table.<br><br>
     *
     * Wrapper for {@link #addEntries(Collection)}.
     *
     * @param tab the tab to add to the current/target tab.
     * @return the target <code>RSTable</code> which the parameter tab was appended to.
     *
     * @see #addEntries(Collection)
     * @since 0.1.0
     */
    public RSTable addTabData(RSTab tab) {
        return addEntries(tab.getEntries().values());
    }

    /**
     * Adds tiles from a list to the current table.<br><br>
     *
     * Calls {@link #setEntry(String, Object)} internally for each <code>NetworkTableEntry</code>.
     * Key/title names are based on the title of the <code>NetworkTableEntry</code>
     * and should be automatically assigned. Do not change these manually.
     *
     * @param ntEntries the entries to add to the current table.
     * @return the current <code>RSTable</code> where the entries were added.
     *
     * @see #setEntry(String, Object)
     * @since 0.1.0
     */
    public RSTable addEntries(Collection<NetworkTableEntry> ntEntries) {
        for (NetworkTableEntry entry : ntEntries) {
            setEntry(entry.getName().substring(13), entry.getValue().getValue());
        }
        return this;
    }

    /**
     * Copy the current table to a passed <code>RSTab</code>.<br><br>
     *
     * Creates a new table with the same table name, options, and entries,
     * just targeted towards a new tab. Does not point the current tab
     * to the passed tab.
     *
     * @param targetTab the target tab to copy the current table onto.
     * @return the new <code>RSTable</code> where the entries were copied to.
     *
     * @since 0.1.0
     */
    public RSTable copyToTab(RSTab targetTab) {
        RSTable table = new RSTable(tableName, targetTab, options).addEntries(entries.values());
        targetTab.addTable(table);
        return table;
    }

    public String getName() {
        return tableName;
    }
}
