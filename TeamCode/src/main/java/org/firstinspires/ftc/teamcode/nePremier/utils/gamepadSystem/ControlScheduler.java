package org.firstinspires.ftc.teamcode.nePremier.utils.gamepadSystem;

public class ControlScheduler extends Builder {
    //instance of single class
    public static ControlScheduler instance = null;

    //private constructor so that it can't be initialized normally
    private ControlScheduler() {}

     /**
    getInstance is a method that returns the ControlScheduler singleton,
    throws IllegalAccessException if scheduler was not initialized automatically
      **/
    public static synchronized ControlScheduler getInstance() throws IllegalAccessException {
        if (instance != null) return instance;
        else throw new IllegalAccessException("what the fuck did you even code");
    }

    /// Required to function, updates the controls
    public void update() {
        controls.forEach(control -> update());
    }

}
