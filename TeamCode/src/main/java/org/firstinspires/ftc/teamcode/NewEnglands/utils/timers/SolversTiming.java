package org.firstinspires.ftc.teamcode.NewEnglands.utils.timers;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

public class SolversTiming extends Timing implements GenericTime {
    private static int length = 1000;
    Timer timer = null;

    public void create() {
        timer = new Timer(length, TimeUnit.MILLISECONDS);
    }

    /**
     * Resets the timer and starts it.
     */
    public void resetThenStart(){
        timer.start();
    }
    public void pause(){
        timer.pause();
    }
    public void resume(){
        timer.resume();
    }
    public boolean checkFinished(){
        return timer.done();
    }
    public void setLength(int lengthInMilliseconds){
        length = lengthInMilliseconds;
    }
}
