package org.firstinspires.ftc.teamcode.utils.commandUtils;

public interface CommandInterface {
    void create();
    void init();
    void loop();
    void stop();
    boolean prioritize = false;
}
