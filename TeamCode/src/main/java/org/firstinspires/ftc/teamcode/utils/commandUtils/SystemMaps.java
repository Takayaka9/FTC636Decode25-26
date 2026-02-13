package org.firstinspires.ftc.teamcode.utils.commandUtils;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

abstract class SystemMaps {
    public Map<String, BaseCommand> commands;
    public Map<String, BaseSubsystem> subsystems;

    public Map<String, BaseCommand> initializingCommands;
    public Map<String, BaseCommand> runningCommands;
    public Map<String, BaseCommand> stoppingCommands;
    public Map<String, BaseCommand> waitingCommands;


    public SystemMaps() {
        commands = new HashMap<>();
        subsystems = new HashMap<>();
        initializingCommands = new HashMap<>();
        runningCommands = new HashMap<>();
        stoppingCommands = new HashMap<>();
        waitingCommands = new LinkedHashMap<>();
    }
    public void addCommand(String name, BaseCommand command) {
        commands.put(name, command);
    }
    public void addSubsystem(String name, BaseSubsystem subsystem) {
        subsystems.put(name, subsystem);
    }
    public BaseCommand getCommand(String name) {
        return commands.get(name);
    }
    public BaseSubsystem getSubsystem(String name) {
        return subsystems.get(name);
    }
}

