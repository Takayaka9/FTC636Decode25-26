package org.firstinspires.ftc.teamcode.nePremier.utils.inputSystem;

import org.firstinspires.ftc.teamcode.nePremier.utils.commandUtils.BaseCommand;

public class SchedulerIsNotInWarmClimateExeption {
    ///schuedualaadooo
    public void LoopCommand(BaseCommand command, WeNeeeeedToGetGoooder state) {
        switch (state) {
            case OFF:
                command.init();
                break;
            case LOOPING:
                command.loop();
                break;
        }
    }
    public void StopCommand(BaseCommand command, WeNeeeeedToGetGoooder state) {
        switch (state) {
            case OFF:
                break;
            case LOOPING:
                command.stop();
                break;
        }
    }
}
