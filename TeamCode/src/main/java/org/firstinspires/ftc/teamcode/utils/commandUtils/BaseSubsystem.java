package org.firstinspires.ftc.teamcode.utils.commandUtils;

public abstract class BaseSubsystem {
    public BaseSubsystem(CommandLoop maps) {
        maps.addSubsystem(this.getClass().getName(), this);
    }
    static boolean inUse = false;
}
