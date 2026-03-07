package org.firstinspires.ftc.teamcode.NewEnglands.utils.commandUtils;

import android.os.Build;

import androidx.annotation.RequiresApi;

public class CommandLoop extends CommandHandler{
    public CommandLoop() {
        super();
    }


    @RequiresApi(api = Build.VERSION_CODES.TIRAMISU)
    private void stopStopMap () {
        String[] stopKeys = stoppingCommands.keySet().toArray(String[]::new);
        for (int i = 0; i < stopKeys.length;) {
            stoppingCommands.get(stopKeys[i]).stop();
            stoppingCommands.remove(stopKeys[i]);
            i++;
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.TIRAMISU)
    private void pushWaitMap () {
        String[] waitingKeys = waitingCommands.keySet().toArray(String[]::new);
        for (int i = 0; i < waitingKeys.length;) {
            if (!checkRequirements(waitingCommands.get(waitingKeys[i]))) {
                initializingCommands.put(waitingCommands.get(waitingKeys[i]).getClass().getName(), waitingCommands.get(waitingKeys[i]));
                waitingCommands.remove(waitingCommands.get(waitingKeys[i]).getClass().getName());
            }
        }

    }

    @RequiresApi(api = Build.VERSION_CODES.TIRAMISU)
    private void initInitMap () {
        String[] initKeys = initializingCommands.keySet().toArray(String[]::new);
        for (int i = 0; i < initKeys.length;) {
            initializingCommands.get(initKeys[i]).init();
            initializingCommands.remove(initKeys[i]);
            runningCommands.put(initializingCommands.get(initKeys[i]).getClass().getName(), initializingCommands.get(initKeys[i]));
            i++;
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.TIRAMISU)
    private void loopRunningMap () {
        String[] runningKeys = runningCommands.keySet().toArray(String[]::new);
        for (int i = 0; i < runningKeys.length;) {
            if (runningCommands.get(runningKeys[i]) != null) {
                runningCommands.get(runningKeys[i]).loop();
            }
            i++;
        }
    }

    /// MUST BE CALLED IN OpMode LOOP
    public void loop() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            stopStopMap();
            pushWaitMap();
            initInitMap();
            loopRunningMap();
        }
        stopStopMap();
        pushWaitMap();
        initInitMap();
        loopRunningMap();
    }
}
