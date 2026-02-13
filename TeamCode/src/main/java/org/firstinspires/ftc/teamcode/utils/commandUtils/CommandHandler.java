package org.firstinspires.ftc.teamcode.utils.commandUtils;

abstract class CommandHandler extends SystemMaps {
    public CommandHandler() {
        super();
    }

    //TODO: change so that it just uses the command's requirement map's keys
    protected boolean checkRequirements(CommandBase command) {
        if (command != null) {
            String[] systemKeys = subsystems.keySet().toArray(String[]::new);
            String[] requirementKeys = command.requirements.keySet().toArray(String[]::new);
            String[] matchedKeys = new String[0];
            for (int i = 0; i < command.requirements.size();) {
                for (int j = 0; j < subsystems.size(); j++) {
                    if (systemKeys[j].equals(requirementKeys[i])) {
                        matchedKeys[i] = systemKeys[j];
                    }
                    j++;
                }
                i++;
            }
            for (int i = 0; i < matchedKeys.length;) {
                if (subsystems.get(matchedKeys[i]).inUse){
                    return true;
                }
                i++;
            }
        }
        return false;
    }

    private boolean readyToStart (CommandBase command) {
        if (!runningCommands.containsKey(command.getClass().getName())
                && !initializingCommands.containsKey(command.getClass().getName())
                && !stoppingCommands.containsKey(command.getClass().getName())
                && !waitingCommands.containsKey(command.getClass().getName())) {
            return true;
        } else {
            return false;
        }
    }

    private boolean readyToStop (CommandBase command) {
        if (!stoppingCommands.containsKey(command.getClass().getName())
                & runningCommands.containsKey(command.getClass().getName())
                & initializingCommands.containsKey(command.getClass().getName())
                & waitingCommands.containsKey(command.getClass().getName())){
            return true;
        } else {
            return false;
        }
    }

    public void runCommand(CommandBase command) {
        if (command != null && readyToStart(command)) {
            if (!checkRequirements(command)) {
                initializingCommands.put(command.getClass().getName(), command);
            } else {
                waitingCommands.put(command.getClass().getName(), command);
            }
        } else return;
    }

    public void stopCommand(CommandBase command) {
        if (command != null && readyToStop(command)) {
            stoppingCommands.put(command.getClass().getName(), command);
            runningCommands.remove(command.getClass().getName());
            initializingCommands.remove(command.getClass().getName());
            waitingCommands.remove(command.getClass().getName());
        } else return;
    }

}
