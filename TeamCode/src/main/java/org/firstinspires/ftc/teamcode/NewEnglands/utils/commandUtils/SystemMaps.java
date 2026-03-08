package org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils;

import android.content.Context;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import dalvik.system.DexFile;

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

    protected Set<Class<?>> getCommandClasses() {
        Enumeration<String> entries = null;
        try {
            @Deprecated
            DexFile dex = new DexFile("dex");
            entries = dex.entries();
        } catch (IOException e) {
            //noinspection CallToPrintStackTrace
            e.printStackTrace();
        }

        Set<Class<?>> classSet = new HashSet<>();
        if (entries != null) {
            while (entries.hasMoreElements()) {
                String className = entries.nextElement();
                if (className.startsWith("org.firstinspires.ftc.teamcode.NewEnglands.robot.commands")) {
                    try {
                        Class<?> clazz = Class.forName(className);
                        classSet.add(clazz);
                    } catch (ClassNotFoundException exeption) {
                        //noinspection CallToPrintStackTrace
                        exeption.printStackTrace();
                    }
                    // check annotations
                }
            }
        }
        return classSet;
    }

    protected void constructCommands() {
        Set<Class<?>> classes = getCommandClasses();

        for (Class<?> clazz : classes) {
            for (Constructor<?> constructor : clazz.getDeclaredConstructors()) {
                if (constructor.isAnnotationPresent(Command.class)) {
                    try {
                        BaseCommand construction = (BaseCommand) constructor.newInstance();
                        addCommand(construction.toString(), construction);
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    } catch (InstantiationException e) {
                        throw new RuntimeException(e);
                    } catch (InvocationTargetException e) {
                        throw new RuntimeException(e);
                    }
                }
            }

        }
    }
}

