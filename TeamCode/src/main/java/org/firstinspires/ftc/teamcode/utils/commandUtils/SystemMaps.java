package org.firstinspires.ftc.teamcode.utils.commandUtils;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

abstract class SystemMaps {
    protected Map<String, BaseCommand> commands;
    protected Map<String, BaseSubsystem> subsystems;

    protected Map<String, BaseCommand> initializingCommands;
    protected Map<String, BaseCommand> runningCommands;
    protected Map<String, BaseCommand> stoppingCommands;
    protected Map<String, BaseCommand> waitingCommands;


    protected SystemMaps() {
        commands = new HashMap<>();
        subsystems = new HashMap<>();
        initializingCommands = new HashMap<>();
        runningCommands = new HashMap<>();
        stoppingCommands = new HashMap<>();
        waitingCommands = new LinkedHashMap<>();
    }
    protected void addCommand(String name, BaseCommand command) {
        commands.put(name, command);
    }
    protected void addSubsystem(String name, BaseSubsystem subsystem) {
        subsystems.put(name, subsystem);
    }
    protected BaseCommand getCommand(String name) {
        return commands.get(name);
    }
    protected BaseSubsystem getSubsystem(String name) {
        return subsystems.get(name);
    }
}

