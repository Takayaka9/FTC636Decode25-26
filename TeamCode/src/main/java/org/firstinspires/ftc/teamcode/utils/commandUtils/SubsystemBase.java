package org.firstinspires.ftc.teamcode.utils.commandUtils;

public abstract class SubsystemBase {
    public SubsystemBase(SystemMaps maps) {
        maps.addSubsystem(this.getClass().getName(), this);
    }
    boolean inUse = false;
}
