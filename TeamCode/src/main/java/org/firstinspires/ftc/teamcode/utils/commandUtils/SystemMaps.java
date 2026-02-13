package org.firstinspires.ftc.teamcode.utils.commandUtils;

import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

abstract class SystemMaps {
    public Map<String, CommandBase> commands;
    public Map<String, SubsystemBase> subsystems;

    public Map<String, CommandBase> initializingCommands;
    public Map<String, CommandBase> runningCommands;
    public Map<String, CommandBase> stoppingCommands;
    public Map<String, CommandBase> waitingCommands;


    public SystemMaps() {
        commands = new HashMap<>();
        subsystems = new HashMap<>();
        initializingCommands = new HashMap<>();
        runningCommands = new HashMap<>();
        stoppingCommands = new HashMap<>();
        waitingCommands = new LinkedHashMap<>();
    }
    public void addCommand(String name, CommandBase command) {
        commands.put(name, command);
    }
    public void addSubsystem(String name, SubsystemBase subsystem) {
        subsystems.put(name, subsystem);
    }
    public CommandBase getCommand(String name) {
        return commands.get(name);
    }
    public SubsystemBase getSubsystem(String name) {
        return subsystems.get(name);
    }
}

