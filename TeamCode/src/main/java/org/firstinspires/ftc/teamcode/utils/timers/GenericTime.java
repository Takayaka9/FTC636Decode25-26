package org.firstinspires.ftc.teamcode.utils.timers;

public interface GenericTime {
    /**
     * Creates Timing object w/ default length of 1000 milliseconds
     */
    void create();
    /**
     * Resets the timer and starts it.
     */
    void resetThenStart();
    /**
     * Pauses the timer.
     */
    void pause();

    /**
     * Starts the timer (for after it has been paused)
     */
    void resume();

    /**
     * Checks if the timer has finished.
     * @return True if the timer has finished, false otherwise
     */
    boolean checkFinished();

    /**
     * Sets the timer length
     * @param lengthInMilliseconds the length of the timer in milliseconds
     */
    void setLength(int lengthInMilliseconds);
}
