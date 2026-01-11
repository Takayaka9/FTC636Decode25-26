package org.firstinspires.ftc.teamcode.RIstates.management.Systems;


public interface Controller {
    boolean isRunning = false;
    /**
     * run in the state's init method
     */
    void init();


    /**
     * run in the state's update method
     */
    void update();

    /**
     * run in the state's end method
     */
    void end();

    /// List of potential controller states
    enum errors {
        RUNNING,
        ErrorCausesNormState,
        ContinueWithError,
    }

    /**
     * run int the states update method to see controller state
     * Use logic to handle errors
     * @return is the current error
     */
    errors updateError();
}

