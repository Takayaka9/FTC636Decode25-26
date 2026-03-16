package org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils;

import android.os.Build;
@Deprecated
abstract class CommandHandler extends SystemMaps {
    public CommandHandler() {
        super();
    }

    //TODO: change so that it just uses the command's requirement map's keys
    protected boolean checkRequirements(BaseCommand command) {
        if (command != null) {
            String[] systemKeys = null;
            systemKeys = subsystems.keySet().toArray(new String[0]);
            String[] requirementKeys = null;
            requirementKeys = command.requirements.keySet().toArray(new String[0]);
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

    private boolean readyToStart (BaseCommand command) {
        if (!runningCommands.containsKey(command.getClass().getName())
                && !initializingCommands.containsKey(command.getClass().getName())
                && !stoppingCommands.containsKey(command.getClass().getName())
                && !waitingCommands.containsKey(command.getClass().getName())) {
            return true;
        } else {
            return false;
        }
    }

    private StopCheck readyToStop (BaseCommand command) {
        if (!stoppingCommands.containsKey(command.getClass().getName())
                & runningCommands.containsKey(command.getClass().getName())
                & initializingCommands.containsKey(command.getClass().getName())){
            return StopCheck.RunToStop;
        } else if (!stoppingCommands.containsKey(command.getClass().getName())
                && waitingCommands.containsKey(command.getClass().getName())) {
            return StopCheck.WaitToStop;
        } else {
            return StopCheck.Stopped;
        }

    }

    public void runCommand(BaseCommand command) {
        if (command != null && readyToStart(command)) {
            if (!checkRequirements(command)) {
                initializingCommands.put(command.getClass().getName(), command);
                emadIsADumbass(command, true);
            } else {
                waitingCommands.put(command.getClass().getName(), command);
            }
        } else return;
    }

    public void stopCommand(BaseCommand command) {
        if (command != null) {
            if (readyToStop(command).equals(StopCheck.RunToStop)) {
                stoppingCommands.put(command.getClass().getName(), command);
                emadIsADumbass(command, false);
                return;

            } else if (readyToStop(command).equals(StopCheck.WaitToStop)) {
                runningCommands.remove(command.getClass().getName());
                initializingCommands.remove(command.getClass().getName());
                waitingCommands.remove(command.getClass().getName());
                stoppingCommands.remove(command.getClass().getName());
                return;
            } else return;
        }
    }

    private void emadIsADumbass(BaseCommand command, Boolean use) {
        if (command != null) {
            String[] systemKeys = null;
            systemKeys = subsystems.keySet().toArray(new String[0]);
            String[] requirementKeys = null;
            requirementKeys = command.requirements.keySet().toArray(new String[0]);
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
                if (subsystems.get(matchedKeys[i]) != null){
                    subsystems.get(matchedKeys[i]).inUse = use;
                }
                i++;
            }
        }
    }

}
