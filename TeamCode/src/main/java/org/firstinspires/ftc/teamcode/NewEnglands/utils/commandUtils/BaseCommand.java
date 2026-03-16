package org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils;

import java.util.HashMap;
import java.util.Map;

public abstract class BaseCommand implements CommandInterface{
    Map<String, BaseSubsystem> requirements;
    public BaseCommand() {
        requirements = new HashMap<>();
    }
//    public void addRequirement(BaseSubsystem... subsystem) {
//        int num = 0;
//        for (BaseSubsystem sub : subsystem) {
//            requirements.put(subsystem.getClass().getName(), subsystem[num]);
//            num++;
//        }
//    }

    public void create() {}
    public void init() {}
    public void loop() {}
    public void stop() {}

}
