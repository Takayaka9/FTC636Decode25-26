package org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils;

public interface CommandInterface {
    void init();
    void loop();
    void stop();
    @Deprecated
    boolean prioritize = false;

}
