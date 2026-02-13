package org.firstinspires.ftc.teamcode.utils.commandUtils;

import java.util.HashMap;
import java.util.Map;

public abstract class CommandBase implements CommandInterface{
    Map<String, SubsystemBase> requirements;
    public CommandBase(SystemMaps maps) {
        requirements = new HashMap<>();
        maps.addCommand(this.getClass().getName(), this);
    }
    public void addRequirement(SubsystemBase... subsystem) {
        int num = 0;
        for (SubsystemBase sub : subsystem) {
            requirements.put(subsystem.getClass().getName(), subsystem[num]);
            num++;
        }
    }

    public void create() {}
    public void init() {}
    public void loop() {}
    public void stop() {}

}
